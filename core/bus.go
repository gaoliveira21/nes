package core

const RAM_SIZE = 64 * 1024 // 64KB

type Bus struct {
	Cpu *CPU6502
	Ram [RAM_SIZE]byte // Fake RAM
}

func (b *Bus) Init() {
	b.Cpu = new(CPU6502)

	b.Cpu.Init()
	b.Cpu.ConnectBus(b)
}

func (b *Bus) Write(addr uint16, data uint8) {
	if addr <= 0xFFFF {
		b.Ram[addr] = data
	}
}

func (b *Bus) Read(addr uint16, readOnly bool) uint8 {
	if addr <= 0xFFFF {
		return b.Ram[addr]
	}

	return 0x00
}
