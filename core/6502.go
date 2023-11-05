package core

type Instruction struct {
	Name      string
	Operation func() uint8
	AddrMode  func() uint8
	Cycles    uint8
}

type Flags6502 struct {
	C uint8 // Carry bit
	Z uint8 // Zero
	I uint8 // Disable Interrupts
	D uint8 // Decimal Mode (unsed in this implementation)
	B uint8 // Break
	U uint8 // Unused
	V uint8 // Overflow
	N uint8 // Negative
}

type CPU6502 struct {
	A      uint8  // Accumulator Register
	X      uint8  // X Register
	Y      uint8  // Y Register
	Sp     uint8  // Stack Pointer (points to location on bus)
	Pc     uint16 // Program Counter
	Status uint8

	bus          *Bus
	flags        *Flags6502
	fetched      uint8
	addrAbs      uint16
	addrRel      uint16
	opcode       uint8
	cycles       uint8
	instructions []*Instruction
}

func (cpu *CPU6502) Init() {
	cpu.flags = &Flags6502{
		C: (1 << 0),
		Z: (1 << 1),
		I: (1 << 2),
		D: (1 << 3),
		B: (1 << 4),
		U: (1 << 5),
		V: (1 << 6),
		N: (1 << 7),
	}

	cpu.instructions = []*Instruction{}
}

func (cpu *CPU6502) ConnectBus(b *Bus) {
	cpu.bus = b
}

func (cpu *CPU6502) Write(addr uint16, data uint8) {
	cpu.bus.Write(addr, data)
}

func (cpu *CPU6502) Read(addr uint16) uint8 {
	return cpu.bus.Read(addr, false)
}

func (cpu *CPU6502) clock() {}
func (cpu *CPU6502) reset() {}
func (cpu *CPU6502) irq()   {}
func (cpu *CPU6502) nmi()   {}

func (cpu *CPU6502) fetch() uint8 {
	return 0
}

// Addressing Modes
func (cpu *CPU6502) imp() uint8 {
	return 0
}

func (cpu *CPU6502) zp0() uint8 {
	return 0
}

func (cpu *CPU6502) zpy() uint8 {
	return 0
}

func (cpu *CPU6502) abs() uint8 {
	return 0
}

func (cpu *CPU6502) aby() uint8 {
	return 0
}

func (cpu *CPU6502) izx() uint8 {
	return 0
}

func (cpu *CPU6502) imm() uint8 {
	return 0
}

func (cpu *CPU6502) zpx() uint8 {
	return 0
}

func (cpu *CPU6502) rel() uint8 {
	return 0
}

func (cpu *CPU6502) abx() uint8 {
	return 0
}

func (cpu *CPU6502) ind() uint8 {
	return 0
}

func (cpu *CPU6502) izy() uint8 {
	return 0
}

// Opcodes Start
func (cpu *CPU6502) adc() uint8 {
	return 0
}

func (cpu *CPU6502) and() uint8 {
	return 0
}

func (cpu *CPU6502) asl() uint8 {
	return 0
}

func (cpu *CPU6502) bcc() uint8 {
	return 0
}

func (cpu *CPU6502) bcs() uint8 {
	return 0
}

func (cpu *CPU6502) beq() uint8 {
	return 0
}

func (cpu *CPU6502) bit() uint8 {
	return 0
}

func (cpu *CPU6502) bmi() uint8 {
	return 0
}

func (cpu *CPU6502) bne() uint8 {
	return 0
}

func (cpu *CPU6502) bpl() uint8 {
	return 0
}

func (cpu *CPU6502) brk() uint8 {
	return 0
}

func (cpu *CPU6502) bvc() uint8 {
	return 0
}

func (cpu *CPU6502) bvs() uint8 {
	return 0
}

func (cpu *CPU6502) clc() uint8 {
	return 0
}

func (cpu *CPU6502) cld() uint8 {
	return 0
}

func (cpu *CPU6502) cli() uint8 {
	return 0
}

func (cpu *CPU6502) clv() uint8 {
	return 0
}

func (cpu *CPU6502) cmp() uint8 {
	return 0
}

func (cpu *CPU6502) cpx() uint8 {
	return 0
}

func (cpu *CPU6502) cpy() uint8 {
	return 0
}

func (cpu *CPU6502) dec() uint8 {
	return 0
}

func (cpu *CPU6502) dex() uint8 {
	return 0
}

func (cpu *CPU6502) dey() uint8 {
	return 0
}

func (cpu *CPU6502) eor() uint8 {
	return 0
}

func (cpu *CPU6502) inc() uint8 {
	return 0
}

func (cpu *CPU6502) inx() uint8 {
	return 0
}

func (cpu *CPU6502) iny() uint8 {
	return 0
}

func (cpu *CPU6502) jmp() uint8 {
	return 0
}

func (cpu *CPU6502) jsr() uint8 {
	return 0
}

func (cpu *CPU6502) lda() uint8 {
	return 0
}

func (cpu *CPU6502) ldx() uint8 {
	return 0
}

func (cpu *CPU6502) ldy() uint8 {
	return 0
}

func (cpu *CPU6502) lsr() uint8 {
	return 0
}

func (cpu *CPU6502) nop() uint8 {
	return 0
}

func (cpu *CPU6502) ora() uint8 {
	return 0
}

func (cpu *CPU6502) pha() uint8 {
	return 0
}

func (cpu *CPU6502) php() uint8 {
	return 0
}

func (cpu *CPU6502) pla() uint8 {
	return 0
}

func (cpu *CPU6502) plp() uint8 {
	return 0
}

func (cpu *CPU6502) rol() uint8 {
	return 0
}

func (cpu *CPU6502) ror() uint8 {
	return 0
}

func (cpu *CPU6502) rti() uint8 {
	return 0
}

func (cpu *CPU6502) rts() uint8 {
	return 0
}

func (cpu *CPU6502) sbc() uint8 {
	return 0
}

func (cpu *CPU6502) sec() uint8 {
	return 0
}

func (cpu *CPU6502) sed() uint8 {
	return 0
}

func (cpu *CPU6502) sei() uint8 {
	return 0
}

func (cpu *CPU6502) sta() uint8 {
	return 0
}

func (cpu *CPU6502) stx() uint8 {
	return 0
}

func (cpu *CPU6502) sty() uint8 {
	return 0
}

func (cpu *CPU6502) tax() uint8 {
	return 0
}

func (cpu *CPU6502) tay() uint8 {
	return 0
}

func (cpu *CPU6502) tsx() uint8 {
	return 0
}

func (cpu *CPU6502) txa() uint8 {
	return 0
}

func (cpu *CPU6502) txs() uint8 {
	return 0
}

func (cpu *CPU6502) tya() uint8 {
	return 0
}

// Used for illegal addresses
func (cpu *CPU6502) xxx() uint8 {
	return 0
}

// ============================================
