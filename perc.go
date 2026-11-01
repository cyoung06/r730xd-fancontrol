//go:build linux

package main

// Drive temperatures through the PERC (megaraid_sas): the same MFI passthrough frame that
// `smartctl -d megaraid,N` sends, issued directly via /dev/megaraid_sas_ioctl_node.

import (
	"encoding/binary"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"unsafe"
)

const (
	megasasIocFirmware = 0xc1944d01 // _IOWR('M', 1, struct megasas_iocpacket), 404 bytes packed
	mfiCmdPdScsiIO     = 0x04
	mfiFrameDirRead    = 0x0010
	iocFrame           = 20  // offsetof(megasas_iocpacket, frame)
	iocSgl             = 148 // offsetof(megasas_iocpacket, sgl): struct iovec[16]
	pthruSgl           = 48  // offsetof(megasas_pthru_frame, sgl)
)

type Drive struct {
	ID     byte // megaraid target id (smartctl's megaraid,N)
	Serial string
	ata    bool // SATA drive behind the controller's SAT layer
}

func (d Drive) String() string { return fmt.Sprintf("%d:%s", d.ID, d.Serial) }

// Perc holds the ioctl node and heap buffers whose addresses go to the kernel.
type Perc struct {
	mu   sync.Mutex
	fd   int
	host uint16
	pkt  []byte
	buf  []byte
}

// megaraidHost is the SCSI host number of the megaraid_sas controller (what smartctl reads
// from the /dev/sdX handle with SCSI_IOCTL_GET_BUS_NUMBER).
func megaraidHost() (uint16, error) {
	names, _ := filepath.Glob("/sys/class/scsi_host/host*/proc_name")
	for _, p := range names {
		if b, err := os.ReadFile(p); err == nil && strings.TrimSpace(string(b)) == "megaraid_sas" {
			if n, err := strconv.Atoi(strings.TrimPrefix(filepath.Base(filepath.Dir(p)), "host")); err == nil {
				return uint16(n), nil
			}
		}
	}
	return 0, errors.New("no megaraid_sas controller")
}

func OpenPerc() (*Perc, error) {
	host, err := megaraidHost()
	if err != nil {
		return nil, err
	}
	fd, err := syscall.Open("/dev/megaraid_sas_ioctl_node", syscall.O_RDWR|syscall.O_CLOEXEC, 0)
	if err != nil {
		return nil, fmt.Errorf("open megaraid ioctl node: %w", err)
	}
	return &Perc{fd: fd, host: host, pkt: make([]byte, 404), buf: make([]byte, 4096)}, nil
}

// scsi sends one read-direction CDB to drive `target` and returns n bytes of data-in.
func (p *Perc) scsi(target byte, cdb []byte, n int) ([]byte, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	pkt, buf, le := p.pkt, p.buf[:n], binary.LittleEndian
	clear(pkt)
	clear(buf)
	le.PutUint16(pkt[0:], p.host)
	le.PutUint32(pkt[4:], pthruSgl) // sgl_off
	le.PutUint32(pkt[8:], 1)        // sge_count
	f := pkt[iocFrame : iocFrame+128]
	f[0] = mfiCmdPdScsiIO
	f[2] = 0xff // cmd_status, overwritten by the driver
	f[4] = target
	f[6] = byte(len(cdb))
	f[7] = 1
	le.PutUint16(f[16:], mfiFrameDirRead)
	le.PutUint16(f[18:], 15) // timeout, s
	le.PutUint32(f[20:], uint32(n))
	copy(f[32:48], cdb)
	le.PutUint32(f[pthruSgl+4:], uint32(n)) // the driver fills in the DMA address
	le.PutUint64(pkt[iocSgl:], uint64(uintptr(unsafe.Pointer(&buf[0]))))
	le.PutUint64(pkt[iocSgl+8:], uint64(n))
	err := ioctl(p.fd, megasasIocFirmware, unsafe.Pointer(&pkt[0]))
	runtime.KeepAlive(buf)
	if err != nil {
		return nil, fmt.Errorf("target %d: %w", target, err)
	}
	if st := f[2]; st != 0 {
		return nil, fmt.Errorf("target %d: cmd_status 0x%02x", target, st)
	}
	return append([]byte(nil), buf...), nil
}

func printable(b []byte) string {
	return strings.Map(func(r rune) rune {
		if r < 0x20 || r > 0x7e {
			return -1
		}
		return r
	}, string(b))
}

// Scan finds the disks behind the controller that report a real temperature (dead or
// unsupported ones are skipped).
func (p *Perc) Scan() []Drive {
	var out []Drive
	for t := 0; t < 64; t++ {
		inq, err := p.scsi(byte(t), []byte{0x12, 0, 0, 0, 96, 0}, 96) // INQUIRY
		if err != nil || inq[0] != 0x00 {                             // connected direct-access device
			continue
		}
		d := Drive{ID: byte(t), Serial: strconv.Itoa(t), ata: strings.TrimSpace(string(inq[8:16])) == "ATA"}
		if v, err := p.scsi(byte(t), []byte{0x12, 1, 0x80, 0, 252, 0}, 252); err == nil { // VPD 80: serial
			if s := strings.TrimSpace(printable(v[4 : 4+min(int(v[3]), 248)])); s != "" {
				d.Serial = s
			}
		}
		if _, ok := p.Temp(d); ok {
			out = append(out, d)
		}
	}
	return out
}

// Temp reads LOG SENSE page 0x0D (smartctl's "Current Drive Temperature"); SATA drives fall back
// to SMART READ DATA attribute 194 (190) through ATA PASS-THROUGH. 0 C = no reading.
func (p *Perc) Temp(d Drive) (float64, bool) {
	if b, err := p.scsi(d.ID, []byte{0x4d, 0, 0x4d, 0, 0, 0, 0, 0, 64, 0}, 64); err == nil {
		if t := logSenseTemp(b); t > 0 {
			return t, true
		}
	}
	if !d.ata {
		return 0, false
	}
	b, err := p.scsi(d.ID, []byte{0x85, 4 << 1, 0x0e, 0, 0xd0, 0, 1, 0, 0, 0, 0x4f, 0, 0xc2, 0, 0xb0, 0}, 512)
	if err != nil {
		return 0, false
	}
	var fallback float64
	for i := 2; i+12 <= 362; i += 12 {
		switch b[i] {
		case 194:
			if b[i+5] > 0 {
				return float64(b[i+5]), true
			}
		case 190:
			fallback = float64(b[i+5])
		}
	}
	return fallback, fallback > 0
}

func logSenseTemp(b []byte) float64 {
	if len(b) < 4 || b[0]&0x3f != 0x0d {
		return 0
	}
	end := min(len(b), 4+(int(b[2])<<8|int(b[3])))
	for i := 4; i+4 <= end; i += 4 + int(b[i+3]) {
		if b[i] == 0 && b[i+1] == 0 { // parameter 0000h: current temperature
			if b[i+3] >= 2 && i+5 < end && b[i+5] != 0xff {
				return float64(b[i+5])
			}
			return 0
		}
	}
	return 0
}
