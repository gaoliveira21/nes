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

	cpu.instructions = []*Instruction{
		{"BRK", cpu.brk, cpu.imm, 7}, {"ORA", cpu.ora, cpu.izx, 6}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 3}, {"ORA", cpu.ora, cpu.zp0, 3}, {"ASL", cpu.asl, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"PHP", cpu.php, cpu.imp, 3}, {"ORA", cpu.ora, cpu.imm, 2}, {"ASL", cpu.asl, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.nop, cpu.imp, 4}, {"ORA", cpu.ora, cpu.abs, 4}, {"ASL", cpu.asl, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BPL", cpu.bpl, cpu.rel, 2}, {"ORA", cpu.ora, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"ORA", cpu.ora, cpu.zpx, 4}, {"ASL", cpu.asl, cpu.zpx, 6}, {"???", cpu.xxx, cpu.imp, 6}, {"CLC", cpu.clc, cpu.imp, 2}, {"ORA", cpu.ora, cpu.aby, 4}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"ORA", cpu.ora, cpu.abx, 4}, {"ASL", cpu.asl, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
		{"JSR", cpu.jsr, cpu.abs, 6}, {"AND", cpu.and, cpu.izx, 6}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"BIT", cpu.bit, cpu.zp0, 3}, {"AND", cpu.and, cpu.zp0, 3}, {"ROL", cpu.rol, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"PLP", cpu.plp, cpu.imp, 4}, {"AND", cpu.and, cpu.imm, 2}, {"ROL", cpu.rol, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"BIT", cpu.bit, cpu.abs, 4}, {"AND", cpu.and, cpu.abs, 4}, {"ROL", cpu.rol, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BMI", cpu.bmi, cpu.rel, 2}, {"AND", cpu.and, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"AND", cpu.and, cpu.zpx, 4}, {"ROL", cpu.rol, cpu.zpx, 5}, {"???", cpu.xxx, cpu.imp, 6}, {"SEC", cpu.sec, cpu.imp, 2}, {"AND", cpu.and, cpu.aby, 4}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"AND", cpu.and, cpu.abx, 4}, {"ROL", cpu.rol, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
		{"RTI", cpu.rti, cpu.imp, 6}, {"EOR", cpu.eor, cpu.izx, 6}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 3}, {"EOR", cpu.eor, cpu.zp0, 3}, {"LSR", cpu.lsr, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"PHA", cpu.pha, cpu.imp, 3}, {"EOR", cpu.eor, cpu.imm, 2}, {"LSR", cpu.lsr, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"JMP", cpu.jmp, cpu.abs, 3}, {"EOR", cpu.eor, cpu.abs, 4}, {"LSR", cpu.lsr, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BVC", cpu.bvc, cpu.rel, 2}, {"EOR", cpu.eor, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"EOR", cpu.eor, cpu.zpx, 4}, {"LSR", cpu.lsr, cpu.zpx, 6}, {"???", cpu.xxx, cpu.imp, 6}, {"CLI", cpu.cli, cpu.imp, 2}, {"EOR", cpu.eor, cpu.aby, 4}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"EOR", cpu.eor, cpu.abx, 4}, {"LSR", cpu.lsr, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
		{"RTS", cpu.rts, cpu.imp, 6}, {"ADC", cpu.adc, cpu.izx, 6}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 3}, {"ADC", cpu.adc, cpu.zp0, 3}, {"ROR", cpu.ror, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"PLA", cpu.pla, cpu.imp, 4}, {"ADC", cpu.adc, cpu.imm, 2}, {"ROR", cpu.ror, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"JMP", cpu.jmp, cpu.ind, 5}, {"ADC", cpu.adc, cpu.abs, 4}, {"ROR", cpu.ror, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BVS", cpu.bvs, cpu.rel, 2}, {"ADC", cpu.adc, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"ADC", cpu.adc, cpu.zpx, 4}, {"ROR", cpu.ror, cpu.zpx, 6}, {"???", cpu.xxx, cpu.imp, 6}, {"SEI", cpu.sei, cpu.imp, 2}, {"ADC", cpu.adc, cpu.aby, 4}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"ADC", cpu.adc, cpu.abx, 4}, {"ROR", cpu.ror, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
	}
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
