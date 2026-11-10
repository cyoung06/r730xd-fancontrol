//go:build linux

package main

// BMC access through the kernel IPMI driver (/dev/ipmi0, include/uapi/linux/ipmi.h): sensor
// repository walk + readings, Dell OEM fan commands and DCMI power limiting. amd64 struct layout.

import (
	"errors"
	"fmt"
	"math"
	"runtime"
	"strings"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

const (
	ipmiSendCommand     = 0x8028690d // _IOR('i', 13, struct ipmi_req)
	ipmiReceiveMsgTrunc = 0xc030690b // _IOWR('i', 11, struct ipmi_recv)
	ipmiSysIfaceAddr    = 0x0c       // IPMI_SYSTEM_INTERFACE_ADDR_TYPE
	ipmiBMCChannel      = 0x0f
	ipmiResponseRecv    = 1 // IPMI_RESPONSE_RECV_TYPE
	ipmiTimeout         = 5 * time.Second
)

type ipmiMsg struct {
	netfn   uint8
	cmd     uint8
	dataLen uint16
	_       uint32
	data    uintptr
}

type ipmiReq struct {
	addr    uintptr
	addrLen uint32
	_       uint32
	msgid   int64
	msg     ipmiMsg
}

type ipmiRecv struct {
	recvType int32
	_        int32
	addr     uintptr
	addrLen  uint32
	_        uint32
	msgid    int64
	msg      ipmiMsg
}

type ipmiSysAddr struct {
	addrType int32
	channel  int16
	lun      uint8
	_        uint8
}

// compile-time checks: these must match the kernel's struct sizes on amd64
var (
	_ [16]struct{} = [unsafe.Sizeof(ipmiMsg{})]struct{}{}
	_ [40]struct{} = [unsafe.Sizeof(ipmiReq{})]struct{}{}
	_ [48]struct{} = [unsafe.Sizeof(ipmiRecv{})]struct{}{}
	_ [8]struct{}  = [unsafe.Sizeof(ipmiSysAddr{})]struct{}{}
)

// BMC is one open /dev/ipmi0. The request/response buffers live in the (heap) struct so the
// addresses handed to the kernel never move.
type BMC struct {
	mu    sync.Mutex
	fd    int
	seq   int64
	addr  ipmiSysAddr
	req   ipmiReq
	recv  ipmiRecv
	raddr [64]byte
	sbuf  [272]byte
	rbuf  [272]byte
}

func OpenBMC() (*BMC, error) {
	for _, p := range []string{"/dev/ipmi0", "/dev/ipmi/0", "/dev/ipmidev/0"} {
		fd, err := syscall.Open(p, syscall.O_RDWR|syscall.O_CLOEXEC, 0)
		if err == nil {
			return &BMC{fd: fd}, nil
		}
		if err != syscall.ENOENT {
			return nil, fmt.Errorf("open %s: %w", p, err)
		}
	}
	return nil, errors.New("no IPMI device (ipmi_si / ipmi_devintf not loaded?)")
}

func ioctl(fd int, req uintptr, arg unsafe.Pointer) error {
	if _, _, e := syscall.Syscall(syscall.SYS_IOCTL, uintptr(fd), req, uintptr(arg)); e != 0 {
		return e
	}
	return nil
}

type pollFd struct {
	fd      int32
	events  int16
	revents int16
}

func pollIn(fd int, d time.Duration) (bool, error) {
	p := pollFd{fd: int32(fd), events: 0x1} // POLLIN
	for {
		n, _, e := syscall.Syscall(syscall.SYS_POLL, uintptr(unsafe.Pointer(&p)), 1, uintptr(d/time.Millisecond+1))
		if e == syscall.EINTR {
			continue
		}
		if e != 0 {
			return false, e
		}
		return n > 0, nil
	}
}

// Raw sends one request to the BMC and returns its completion code and response data.
func (b *BMC) Raw(lun, netfn, cmd byte, data []byte) (byte, []byte, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.seq++
	b.addr = ipmiSysAddr{addrType: ipmiSysIfaceAddr, channel: ipmiBMCChannel, lun: lun}
	n := copy(b.sbuf[:], data)
	b.req = ipmiReq{addr: uintptr(unsafe.Pointer(&b.addr)), addrLen: uint32(unsafe.Sizeof(b.addr)), msgid: b.seq,
		msg: ipmiMsg{netfn: netfn, cmd: cmd, dataLen: uint16(n), data: uintptr(unsafe.Pointer(&b.sbuf[0]))}}
	if err := ioctl(b.fd, ipmiSendCommand, unsafe.Pointer(&b.req)); err != nil {
		return 0, nil, fmt.Errorf("ipmi %02x/%02x send: %w", netfn, cmd, err)
	}
	deadline := time.Now().Add(ipmiTimeout)
	for {
		left := time.Until(deadline)
		if left <= 0 {
			return 0, nil, fmt.Errorf("ipmi %02x/%02x: no response in %v", netfn, cmd, ipmiTimeout)
		}
		if ready, err := pollIn(b.fd, left); err != nil {
			return 0, nil, fmt.Errorf("ipmi poll: %w", err)
		} else if !ready {
			continue
		}
		b.recv = ipmiRecv{addr: uintptr(unsafe.Pointer(&b.raddr[0])), addrLen: uint32(len(b.raddr)),
			msg: ipmiMsg{dataLen: uint16(len(b.rbuf)), data: uintptr(unsafe.Pointer(&b.rbuf[0]))}}
		err := ioctl(b.fd, ipmiReceiveMsgTrunc, unsafe.Pointer(&b.recv))
		runtime.KeepAlive(b)
		if err == syscall.EAGAIN {
			continue
		}
		if err != nil && err != syscall.EMSGSIZE {
			return 0, nil, fmt.Errorf("ipmi %02x/%02x receive: %w", netfn, cmd, err)
		}
		if b.recv.recvType != ipmiResponseRecv || b.recv.msgid != b.seq {
			continue // an event, or the late reply to an earlier request that timed out
		}
		m := int(b.recv.msg.dataLen)
		if m < 1 {
			return 0, nil, fmt.Errorf("ipmi %02x/%02x: empty response", netfn, cmd)
		}
		return b.rbuf[0], append([]byte(nil), b.rbuf[1:m]...), nil
	}
}

// Cmd is Raw on LUN 0 that treats any non-zero completion code as an error.
func (b *BMC) Cmd(netfn, cmd byte, data ...byte) ([]byte, error) {
	cc, d, err := b.Raw(0, netfn, cmd, data)
	if err == nil && cc != 0 {
		err = fmt.Errorf("ipmi %02x/%02x: completion code 0x%02x", netfn, cmd, cc)
	}
	return d, err
}

// ---------- Dell OEM fan control ----------

func (b *BMC) FanAuto() error {
	_, err := b.Cmd(0x30, 0x30, 0x01, 0x01)
	return err
}

func (b *BMC) FanDuty(pct int) error {
	pct = max(0, min(100, pct))
	if _, err := b.Cmd(0x30, 0x30, 0x01, 0x00); err != nil {
		return err
	}
	_, err := b.Cmd(0x30, 0x30, 0x02, 0xff, byte(pct))
	return err
}

// ---------- DCMI power limit ----------

type PowerLimit struct {
	Active bool
	Watts  int
	raw    []byte // group id, reserved x2, exception action, limit x2, correction ms x4, reserved x2, sampling s x2
}

func (b *BMC) GetPowerLimit() (PowerLimit, error) {
	cc, d, err := b.Raw(0, 0x2c, 0x03, []byte{0xdc, 0, 0})
	if err != nil {
		return PowerLimit{}, err
	}
	if (cc != 0 && cc != 0x80) || len(d) < 14 || d[0] != 0xdc { // 0x80 = no active limit
		return PowerLimit{}, fmt.Errorf("dcmi get power limit: cc 0x%02x, % x", cc, d)
	}
	return PowerLimit{Active: cc == 0, Watts: int(d[4]) | int(d[5])<<8, raw: d}, nil
}

// SetPowerCap sets and activates the limit, keeping the correction time and sampling period the
// iDRAC already has (exception action forced to "no action", never hard power-off), then reads it back. Never 'deactivate': on iDRAC8 a later
// activate silently fails, so "no cap" is a high limit instead.
func (b *BMC) SetPowerCap(watts int) error {
	cur, err := b.GetPowerLimit()
	if err != nil {
		return err
	}
	r := cur.raw
	if _, err := b.Cmd(0x2c, 0x04, 0xdc, 0, 0, 0, 0x00, byte(watts), byte(watts>>8),
		r[6], r[7], r[8], r[9], 0, 0, r[12], r[13]); err != nil {
		return fmt.Errorf("dcmi set power limit %d W: %w", watts, err)
	}
	if _, err := b.Cmd(0x2c, 0x05, 0xdc, 0x01, 0, 0); err != nil {
		return fmt.Errorf("dcmi activate power limit: %w", err)
	}
	got, err := b.GetPowerLimit()
	if err != nil {
		return err
	}
	if !got.Active || got.Watts != watts {
		return fmt.Errorf("power cap %d W not applied: iDRAC reports %d W, active=%v", watts, got.Watts, got.Active)
	}
	return nil
}

// ---------- sensors ----------

const (
	sensorTemp    = 0x01
	sensorCurrent = 0x03 // "Pwr Consumption" (watts) is a current-type sensor on the iDRAC
	sensorFan     = 0x04
)

// Sensor is a full sensor record (SDR type 0x01) with its reading conversion factors.
type Sensor struct {
	Num, LUN, Type byte
	Entity, Inst   byte
	Name           string
	format         byte // analog data format: 0 unsigned, 1 one's complement, 2 two's complement
	m, b           float64
	bexp, rexp     int
	lin            byte
}

func signExtend(v, bits int) int {
	if v&(1<<(bits-1)) != 0 {
		return v - 1<<bits
	}
	return v
}

func (b *BMC) reserveSDR() (uint16, error) {
	d, err := b.Cmd(0x0a, 0x22)
	if err != nil || len(d) < 2 {
		return 0, fmt.Errorf("reserve SDR repository: %v", err)
	}
	return uint16(d[0]) | uint16(d[1])<<8, nil
}

// getSDR reads one record: the 5-byte header, then the body in 16-byte pieces (the iDRAC refuses
// large reads). Returns the record and the next record id (0xffff = last).
func (b *BMC) getSDR(resv *uint16, id uint16) ([]byte, uint16, error) {
	read := func(off, n byte) ([]byte, uint16, error) {
		for try := 0; try < 3; try++ {
			cc, d, err := b.Raw(0, 0x0a, 0x23, []byte{byte(*resv), byte(*resv >> 8), byte(id), byte(id >> 8), off, n})
			if err != nil {
				return nil, 0, err
			}
			if cc == 0xc5 { // reservation cancelled (the repository changed): reserve again
				if *resv, err = b.reserveSDR(); err != nil {
					return nil, 0, err
				}
				continue
			}
			if cc != 0 || len(d) < 2 {
				return nil, 0, fmt.Errorf("get SDR %04x: cc 0x%02x", id, cc)
			}
			return d[2:], uint16(d[0]) | uint16(d[1])<<8, nil
		}
		return nil, 0, fmt.Errorf("get SDR %04x: reservation keeps getting cancelled", id)
	}
	hdr, next, err := read(0, 5)
	if err != nil {
		return nil, 0, err
	}
	if len(hdr) < 5 {
		return nil, 0, fmt.Errorf("get SDR %04x: short header", id)
	}
	rec, total := append([]byte(nil), hdr...), 5+int(hdr[4])
	if total > 255 { // offsets are one byte; real records are <= 64
		return nil, 0, fmt.Errorf("get SDR %04x: record length %d", id, total)
	}
	for len(rec) < total {
		part, _, err := read(byte(len(rec)), byte(min(16, total-len(rec))))
		if err != nil {
			return nil, 0, err
		}
		if len(part) == 0 {
			return nil, 0, fmt.Errorf("get SDR %04x: empty read", id)
		}
		rec = append(rec, part...)
	}
	return rec, next, nil
}

// Sensors walks the sensor data repository and returns the temperature and fan sensors and the
// power reading.
func (b *BMC) Sensors() ([]Sensor, error) {
	resv, err := b.reserveSDR()
	if err != nil {
		return nil, err
	}
	var out []Sensor
	for id, n := uint16(0), 0; id != 0xffff; n++ {
		if n > 1000 {
			return nil, errors.New("SDR walk does not end")
		}
		rec, next, err := b.getSDR(&resv, id)
		if err != nil {
			return nil, err
		}
		id = next
		if len(rec) < 48 || rec[3] != 0x01 || rec[20]>>6 == 3 { // full records with an analog reading
			continue
		}
		s := Sensor{Num: rec[7], LUN: rec[6] & 3, Type: rec[12], Entity: rec[8], Inst: rec[9] & 0x7f,
			format: rec[20] >> 6, lin: rec[23] & 0x7f,
			m:    float64(signExtend(int(rec[24])|int(rec[25]&0xc0)<<2, 10)),
			b:    float64(signExtend(int(rec[26])|int(rec[27]&0xc0)<<2, 10)),
			rexp: signExtend(int(rec[29]>>4), 4), bexp: signExtend(int(rec[29]&0x0f), 4)}
		if l := int(rec[47] & 0x1f); len(rec) >= 48+l {
			s.Name = strings.TrimRight(string(rec[48:48+l]), "\x00 ")
		}
		if s.Type == sensorTemp || s.Type == sensorFan || (s.Type == sensorCurrent && strings.Contains(s.Name, "Pwr")) {
			out = append(out, s)
		}
	}
	return out, nil
}

// Reading returns the converted value, or ok=false with why (the BMC's answer: completion code,
// reading unavailable, scanning disabled) when the BMC has no reading for the sensor.
func (b *BMC) Reading(s Sensor) (v float64, ok bool, why string, err error) {
	cc, d, err := b.Raw(s.LUN, 0x04, 0x2d, []byte{s.Num})
	if err != nil {
		return 0, false, "", err
	}
	switch { // status byte d[1]: bit 6 = scanning enabled, bit 5 = reading unavailable
	case cc != 0:
		return 0, false, fmt.Sprintf("completion code 0x%02x", cc), nil
	case len(d) < 2:
		return 0, false, fmt.Sprintf("short reply % x", d), nil
	case d[1]&0x20 != 0:
		return 0, false, fmt.Sprintf("reading unavailable, status 0x%02x", d[1]), nil
	case d[1]&0x40 == 0:
		return 0, false, fmt.Sprintf("scanning disabled, status 0x%02x", d[1]), nil
	}
	var x float64
	switch s.format {
	case 1:
		if d[0]&0x80 != 0 {
			x = -float64(^d[0] & 0x7f)
		} else {
			x = float64(d[0])
		}
	case 2:
		x = float64(int8(d[0]))
	default:
		x = float64(d[0])
	}
	y := (s.m*x + s.b*math.Pow10(s.bexp)) * math.Pow10(s.rexp)
	switch s.lin {
	case 1:
		y = math.Log(y)
	case 2:
		y = math.Log10(y)
	case 3:
		y = math.Log2(y)
	case 4:
		y = math.Exp(y)
	case 5:
		y = math.Pow(10, y)
	case 6:
		y = math.Exp2(y)
	case 7:
		y = 1 / y
	case 8:
		y = y * y
	case 9:
		y = y * y * y
	case 10:
		y = math.Sqrt(y)
	case 11:
		y = math.Cbrt(y)
	}
	return y, true, "", nil
}
