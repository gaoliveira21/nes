package core

import (
	"log"
)

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
	isImp        bool
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
		{"???", cpu.nop, cpu.imp, 2}, {"STA", cpu.sta, cpu.izx, 6}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 6}, {"STY", cpu.sty, cpu.zp0, 3}, {"STA", cpu.sta, cpu.zp0, 3}, {"STX", cpu.stx, cpu.zp0, 3}, {"???", cpu.xxx, cpu.imp, 3}, {"DEY", cpu.dey, cpu.imp, 2}, {"???", cpu.nop, cpu.imp, 2}, {"TXA", cpu.txa, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"STY", cpu.sty, cpu.abs, 4}, {"STA", cpu.sta, cpu.abs, 4}, {"STX", cpu.stx, cpu.abs, 4}, {"???", cpu.xxx, cpu.imp, 4},
		{"BCC", cpu.bcc, cpu.rel, 2}, {"STA", cpu.sta, cpu.izy, 6}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 6}, {"STY", cpu.sty, cpu.zpx, 4}, {"STA", cpu.sta, cpu.zpx, 4}, {"STX", cpu.stx, cpu.zpy, 4}, {"???", cpu.xxx, cpu.imp, 4}, {"TYA", cpu.tya, cpu.imp, 2}, {"STA", cpu.sta, cpu.aby, 5}, {"TXS", cpu.txs, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 5}, {"???", cpu.nop, cpu.imp, 5}, {"STA", cpu.sta, cpu.abx, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"???", cpu.xxx, cpu.imp, 5},
		{"LDY", cpu.ldy, cpu.imm, 2}, {"LDA", cpu.lda, cpu.izx, 6}, {"LDX", cpu.ldx, cpu.imm, 2}, {"???", cpu.xxx, cpu.imp, 6}, {"LDY", cpu.ldy, cpu.zp0, 3}, {"LDA", cpu.lda, cpu.zp0, 3}, {"LDX", cpu.ldx, cpu.zp0, 3}, {"???", cpu.xxx, cpu.imp, 3}, {"TAY", cpu.tay, cpu.imp, 2}, {"LDA", cpu.lda, cpu.imm, 2}, {"TAX", cpu.tax, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"LDY", cpu.ldy, cpu.abs, 4}, {"LDA", cpu.lda, cpu.abs, 4}, {"LDX", cpu.ldx, cpu.abs, 4}, {"???", cpu.xxx, cpu.imp, 4},
		{"BCS", cpu.bcs, cpu.rel, 2}, {"LDA", cpu.lda, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 5}, {"LDY", cpu.ldy, cpu.zpx, 4}, {"LDA", cpu.lda, cpu.zpx, 4}, {"LDX", cpu.ldx, cpu.zpy, 4}, {"???", cpu.xxx, cpu.imp, 4}, {"CLV", cpu.clv, cpu.imp, 2}, {"LDA", cpu.lda, cpu.aby, 4}, {"TSX", cpu.tsx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 4}, {"LDY", cpu.ldy, cpu.abx, 4}, {"LDA", cpu.lda, cpu.abx, 4}, {"LDX", cpu.ldx, cpu.aby, 4}, {"???", cpu.xxx, cpu.imp, 4},
		{"CPY", cpu.cpy, cpu.imm, 2}, {"CMP", cpu.cmp, cpu.izx, 6}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"CPY", cpu.cpy, cpu.zp0, 3}, {"CMP", cpu.cmp, cpu.zp0, 3}, {"DEC", cpu.dec, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"INY", cpu.iny, cpu.imp, 2}, {"CMP", cpu.cmp, cpu.imm, 2}, {"DEX", cpu.dex, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 2}, {"CPY", cpu.cpy, cpu.abs, 4}, {"CMP", cpu.cmp, cpu.abs, 4}, {"DEC", cpu.dec, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BNE", cpu.bne, cpu.rel, 2}, {"CMP", cpu.cmp, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"CMP", cpu.cmp, cpu.zpx, 4}, {"DEC", cpu.dec, cpu.zpx, 6}, {"???", cpu.xxx, cpu.imp, 6}, {"CLD", cpu.cld, cpu.imp, 2}, {"CMP", cpu.cmp, cpu.aby, 4}, {"NOP", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"CMP", cpu.cmp, cpu.abx, 4}, {"DEC", cpu.dec, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
		{"CPX", cpu.cpx, cpu.imm, 2}, {"SBC", cpu.sbc, cpu.izx, 6}, {"???", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"CPX", cpu.cpx, cpu.zp0, 3}, {"SBC", cpu.sbc, cpu.zp0, 3}, {"INC", cpu.inc, cpu.zp0, 5}, {"???", cpu.xxx, cpu.imp, 5}, {"INX", cpu.inx, cpu.imp, 2}, {"SBC", cpu.sbc, cpu.imm, 2}, {"NOP", cpu.nop, cpu.imp, 2}, {"???", cpu.sbc, cpu.imp, 2}, {"CPX", cpu.cpx, cpu.abs, 4}, {"SBC", cpu.sbc, cpu.abs, 4}, {"INC", cpu.inc, cpu.abs, 6}, {"???", cpu.xxx, cpu.imp, 6},
		{"BEQ", cpu.beq, cpu.rel, 2}, {"SBC", cpu.sbc, cpu.izy, 5}, {"???", cpu.xxx, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 8}, {"???", cpu.nop, cpu.imp, 4}, {"SBC", cpu.sbc, cpu.zpx, 4}, {"INC", cpu.inc, cpu.zpx, 6}, {"???", cpu.xxx, cpu.imp, 6}, {"SED", cpu.sed, cpu.imp, 2}, {"SBC", cpu.sbc, cpu.aby, 4}, {"NOP", cpu.nop, cpu.imp, 2}, {"???", cpu.xxx, cpu.imp, 7}, {"???", cpu.nop, cpu.imp, 4}, {"SBC", cpu.sbc, cpu.abx, 4}, {"INC", cpu.inc, cpu.abx, 7}, {"???", cpu.xxx, cpu.imp, 7},
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

func (cpu *CPU6502) Exec() {
	cpu.clock()
}

func (cpu *CPU6502) clock() {
	if cpu.cycles == 0 {
		cpu.opcode = cpu.Read(cpu.Pc)
		cpu.Pc++

		instruction := cpu.instructions[cpu.opcode]

		log.Printf("[0x%X] %s (PC - 0x%X) Z: %d", cpu.opcode, instruction.Name, cpu.Pc, cpu.getFlag(cpu.flags.Z))

		cpu.cycles = instruction.Cycles

		addrAdditionalCycle := instruction.AddrMode()
		opAdditionalCycle := instruction.Operation()

		cpu.isImp = false
		cpu.cycles += (addrAdditionalCycle & opAdditionalCycle)
	}

	cpu.cycles--
}

func (cpu *CPU6502) Reset() {
	cpu.A = 0
	cpu.X = 0
	cpu.Y = 0
	cpu.Sp = 0xFD // Stack Pointer is decremented by 3 (0x0100 - 0x0003 = 0xFD)
	cpu.Status = 0x00 | cpu.flags.U

	cpu.addrAbs = 0xFFFC
	lb := uint16(cpu.Read(cpu.addrAbs))
	hb := uint16(cpu.Read(cpu.addrAbs + 1))

	cpu.Pc = (hb << 8) | lb

	cpu.addrRel = 0x0000
	cpu.addrAbs = 0x0000
	cpu.fetched = 0x00

	cpu.cycles = 8
}

func (cpu *CPU6502) IRQ() {
	if cpu.getFlag(cpu.flags.I) == 0 {
		cpu.Write(0x0100+uint16(cpu.Sp), uint8((cpu.Pc>>8)&0x00FF))
		cpu.Sp--

		cpu.Write(0x0100+uint16(cpu.Sp), uint8(cpu.Pc&0x00FF))
		cpu.Sp--

		cpu.setFlag(cpu.flags.B, false)
		cpu.setFlag(cpu.flags.U, true)
		cpu.setFlag(cpu.flags.I, true)
		cpu.Write(0x0100+uint16(cpu.Sp), cpu.Status)
		cpu.Sp--

		cpu.addrAbs = 0xFFFE
		lb := uint16(cpu.Read(cpu.addrAbs))
		hb := uint16(cpu.Read(cpu.addrAbs + 1))

		cpu.Pc = (hb << 8) | lb

		cpu.cycles = 7
	}
}

func (cpu *CPU6502) NMI() {
	cpu.Write(0x0100+uint16(cpu.Sp), uint8((cpu.Pc>>8)&0x00FF))
	cpu.Sp--

	cpu.Write(0x0100+uint16(cpu.Sp), uint8(cpu.Pc&0x00FF))
	cpu.Sp--

	cpu.setFlag(cpu.flags.B, false)
	cpu.setFlag(cpu.flags.U, true)
	cpu.setFlag(cpu.flags.I, true)
	cpu.Write(0x0100+uint16(cpu.Sp), cpu.Status)
	cpu.Sp--

	cpu.addrAbs = 0xFFFA
	lb := uint16(cpu.Read(cpu.addrAbs))
	hb := uint16(cpu.Read(cpu.addrAbs + 1))

	cpu.Pc = (hb << 8) | lb

	cpu.cycles = 8
}

func (cpu *CPU6502) fetch() uint8 {
	if !cpu.isImp {
		cpu.fetched = cpu.Read(cpu.addrAbs)
	}

	return cpu.fetched
}

func (cpu *CPU6502) getFlag(flag uint8) uint8 {
	if (cpu.Status & flag) > 0 {
		return 1
	}

	return 0
}

func (cpu *CPU6502) setFlag(flag uint8, value bool) {
	if value {
		cpu.Status |= flag
	} else {
		cpu.Status &= ^flag
	}
}

// Addressing Modes
func (cpu *CPU6502) imp() uint8 {
	cpu.fetched = cpu.A
	cpu.isImp = true

	return 0
}

func (cpu *CPU6502) imm() uint8 {
	cpu.addrAbs = cpu.Pc
	cpu.Pc++

	return 0
}

func (cpu *CPU6502) zp0() uint8 {
	b := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	cpu.addrAbs = b & 0x00FF

	return 0
}

func (cpu *CPU6502) zpx() uint8 {
	b := uint16(cpu.Read(cpu.Pc)) + uint16(cpu.X)
	cpu.Pc++

	cpu.addrAbs = b & 0x00FF

	return 0
}

func (cpu *CPU6502) zpy() uint8 {
	b := uint16(cpu.Read(cpu.Pc)) + uint16(cpu.Y)
	cpu.Pc++

	cpu.addrAbs = b & 0x00FF

	return 0
}

func (cpu *CPU6502) abs() uint8 {
	lb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	hb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	cpu.addrAbs = (hb << 8) | lb

	return 0
}

func (cpu *CPU6502) abx() uint8 {
	lb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	hb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	pg := hb << 8

	cpu.addrAbs = pg | lb
	cpu.addrAbs += uint16(cpu.X)

	// Incrementing addrAbs might change page, so we'll need an extra cycle
	if (cpu.addrAbs & 0xFF00) != pg {
		return 1
	}

	return 0
}

func (cpu *CPU6502) aby() uint8 {
	lb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	hb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	pg := hb << 8

	cpu.addrAbs = pg | lb
	cpu.addrAbs += uint16(cpu.Y)

	if (cpu.addrAbs & 0xFF00) != pg {
		return 1
	}

	return 0
}

func (cpu *CPU6502) ind() uint8 {
	lb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	hb := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	ptr := (hb << 8) | lb

	newHb := uint16(cpu.Read(ptr+1)) << 8
	newLb := uint16(cpu.Read(ptr))

	// Simulate page boundary hardware bug
	if lb == 0x00FF {
		newHb = uint16(cpu.Read(ptr&0xFF00)) << 8
	}

	cpu.addrAbs = newHb | newLb

	return 0
}

func (cpu *CPU6502) izx() uint8 {
	b := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	lb := uint16(cpu.Read((b + uint16(cpu.X)) & 0x00FF))
	hb := uint16(cpu.Read((b + uint16(cpu.X) + 1) & 0x00FF))

	cpu.addrAbs = (hb << 8) | lb

	return 0
}

func (cpu *CPU6502) izy() uint8 {
	b := uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	lb := uint16(cpu.Read(b & 0x00FF))
	hb := uint16(cpu.Read((b + 1) & 0x00FF))

	pg := hb << 8

	cpu.addrAbs = pg | lb
	cpu.addrAbs += uint16(cpu.Y)

	if (cpu.addrAbs & 0xFF00) != pg {
		return 1
	}

	return 0
}

func (cpu *CPU6502) rel() uint8 {
	cpu.addrRel = uint16(cpu.Read(cpu.Pc))
	cpu.Pc++

	if (cpu.addrRel & 0x80) > 0 {
		cpu.addrRel |= 0xFF00
	}

	return 0
}

// Opcodes Start
func (cpu *CPU6502) adc() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.A) + uint16(cpu.fetched) + uint16(cpu.getFlag(cpu.flags.C))
	cpu.setFlag(cpu.flags.C, tmp > 255)
	cpu.setFlag(cpu.flags.Z, (tmp&0x00FF) == 0)
	cpu.setFlag(cpu.flags.N, (tmp&0x80) > 0)

	cpu.setFlag(
		cpu.flags.V,
		((^(uint16(cpu.A)^uint16(cpu.fetched))&(uint16(cpu.A)^tmp))&0x0080) > 0,
	)

	cpu.A = uint8(tmp & 0x00FF)

	return 1
}

func (cpu *CPU6502) and() uint8 {
	cpu.fetch()
	cpu.A = cpu.A & cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, (cpu.A&0x80) > 0)

	return 1
}

func (cpu *CPU6502) asl() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.fetched) << 1

	cpu.setFlag(cpu.flags.Z, (tmp&0x00FF) == 0x00)
	cpu.setFlag(cpu.flags.N, tmp&0x80 > 0)
	cpu.setFlag(cpu.flags.C, (tmp&0xFF00) > 0)

	if cpu.isImp {
		cpu.A = uint8(tmp & 0x00FF)
	} else {
		cpu.Write(cpu.addrAbs, uint8(tmp&0x00FF))
	}

	return 0
}

func (cpu *CPU6502) bcc() uint8 {
	if cpu.getFlag(cpu.flags.C) == 0 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) bcs() uint8 {
	if cpu.getFlag(cpu.flags.C) == 1 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) beq() uint8 {
	if cpu.getFlag(cpu.flags.Z) == 1 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) bit() uint8 {
	cpu.fetch()

	tmp := cpu.A & cpu.fetched

	cpu.setFlag(cpu.flags.Z, tmp == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.fetched&(1<<7) > 0)
	cpu.setFlag(cpu.flags.V, cpu.fetched&(1<<6) > 0)

	return 0
}

func (cpu *CPU6502) bmi() uint8 {
	if cpu.getFlag(cpu.flags.N) == 1 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) bne() uint8 {
	if cpu.getFlag(cpu.flags.Z) == 0 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) bpl() uint8 {
	if cpu.getFlag(cpu.flags.N) == 0 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) brk() uint8 {
	cpu.Pc++
	cpu.setFlag(cpu.flags.I, true)

	cpu.Write(0x0100+uint16(cpu.Sp), uint8((cpu.Pc)>>8)&0x00FF)
	cpu.Sp--

	cpu.Write(0x0100+uint16(cpu.Sp), uint8(cpu.Pc)&0x00FF)
	cpu.Sp--

	cpu.setFlag(cpu.flags.B, true)
	cpu.Write(0x0100+uint16(cpu.Sp), cpu.Status)
	cpu.Sp--
	cpu.setFlag(cpu.flags.B, false)

	lb := uint16(cpu.Read(0xFFFE))
	hb := uint16(cpu.Read(0xFFFF))

	cpu.Pc = (hb << 8) | lb

	return 0
}

func (cpu *CPU6502) bvc() uint8 {
	if cpu.getFlag(cpu.flags.V) == 0 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) bvs() uint8 {
	if cpu.getFlag(cpu.flags.V) == 1 {
		cpu.cycles++
		cpu.addrAbs = cpu.Pc + cpu.addrRel

		if (cpu.addrAbs & 0xFF00) != (cpu.Pc & 0xFF00) {
			cpu.cycles++
		}

		cpu.Pc = cpu.addrAbs
	}

	return 0
}

func (cpu *CPU6502) clc() uint8 {
	cpu.setFlag(cpu.flags.C, false)
	return 0
}

func (cpu *CPU6502) cld() uint8 {
	cpu.setFlag(cpu.flags.D, false)
	return 0
}

func (cpu *CPU6502) cli() uint8 {
	cpu.setFlag(cpu.flags.I, false)
	return 0
}

func (cpu *CPU6502) clv() uint8 {
	cpu.setFlag(cpu.flags.V, false)
	return 0
}

func (cpu *CPU6502) cmp() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.A) - uint16(cpu.fetched)

	cpu.setFlag(cpu.flags.Z, tmp&0x00FF == 0x0000)
	cpu.setFlag(cpu.flags.N, tmp&0x0080 > 0)
	cpu.setFlag(cpu.flags.C, cpu.A >= cpu.fetched)

	return 1
}

func (cpu *CPU6502) cpx() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.X) - uint16(cpu.fetched)

	cpu.setFlag(cpu.flags.Z, tmp&0x00FF == 0x00)
	cpu.setFlag(cpu.flags.N, tmp&0x0080 > 0)
	cpu.setFlag(cpu.flags.C, cpu.X >= cpu.fetched)

	return 0
}

func (cpu *CPU6502) cpy() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.Y) - uint16(cpu.fetched)

	cpu.setFlag(cpu.flags.Z, tmp&0x00FF == 0x0000)
	cpu.setFlag(cpu.flags.N, tmp&0x0080 > 0)
	cpu.setFlag(cpu.flags.C, cpu.Y >= cpu.fetched)

	return 0
}

func (cpu *CPU6502) dec() uint8 {
	cpu.fetch()

	tmp := cpu.fetched - 1
	cpu.Write(cpu.addrAbs, tmp)

	cpu.setFlag(cpu.flags.Z, tmp == 0x00)
	cpu.setFlag(cpu.flags.N, tmp&0x80 > 0)

	return 0
}

func (cpu *CPU6502) dex() uint8 {
	cpu.X--

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 0
}

func (cpu *CPU6502) dey() uint8 {
	cpu.Y--

	cpu.setFlag(cpu.flags.Z, cpu.Y == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.Y&0x80 > 0)

	return 0
}

func (cpu *CPU6502) eor() uint8 {
	cpu.fetch()

	cpu.A = cpu.A ^ cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 1
}

func (cpu *CPU6502) inc() uint8 {
	cpu.fetch()

	tmp := cpu.fetched + 1

	cpu.Write(cpu.addrAbs, tmp)

	cpu.setFlag(cpu.flags.Z, tmp == 0x00)
	cpu.setFlag(cpu.flags.N, tmp&0x80 > 0)

	return 0
}

func (cpu *CPU6502) inx() uint8 {
	cpu.X++

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 0
}

func (cpu *CPU6502) iny() uint8 {
	cpu.Y++

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 0
}

func (cpu *CPU6502) jmp() uint8 {
	cpu.Pc = cpu.addrAbs

	return 0
}

func (cpu *CPU6502) jsr() uint8 {
	cpu.Pc--

	cpu.Write(0x0100+uint16(cpu.Sp), uint8((cpu.Pc>>8)&0x00FF))
	cpu.Sp--

	cpu.Write(0x0100+uint16(cpu.Sp), uint8(cpu.Pc&0x00FF))
	cpu.Sp--

	cpu.Pc = cpu.addrAbs

	return 0
}

func (cpu *CPU6502) lda() uint8 {
	cpu.fetch()

	cpu.A = cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 1
}

func (cpu *CPU6502) ldx() uint8 {
	cpu.fetch()

	cpu.X = cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 1
}
func (cpu *CPU6502) ldy() uint8 {
	cpu.fetch()

	cpu.Y = cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.Y == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.Y&0x80 > 0)

	return 1
}

func (cpu *CPU6502) lsr() uint8 {
	cpu.fetch()

	tmp := cpu.fetched >> 1

	cpu.setFlag(cpu.flags.N, false)
	cpu.setFlag(cpu.flags.Z, tmp == 0x00)
	cpu.setFlag(cpu.flags.C, cpu.fetched&0x01 > 0)

	if cpu.isImp {
		cpu.A = tmp
	} else {
		cpu.Write(cpu.addrAbs, tmp)
	}

	return 0
}

func (cpu *CPU6502) nop() uint8 {
	switch cpu.opcode {
	case 0x1C:
	case 0x3C:
	case 0x5C:
	case 0x7C:
	case 0xDC:
	case 0xFC:
		return 1
	}

	return 0
}

func (cpu *CPU6502) ora() uint8 {
	cpu.fetch()

	cpu.A |= cpu.fetched

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 1
}

func (cpu *CPU6502) pha() uint8 {
	cpu.Write(0x0100+uint16(cpu.Sp), cpu.A)
	cpu.Sp--

	return 0
}

func (cpu *CPU6502) php() uint8 {
	cpu.Write(0x0100+uint16(cpu.Sp), cpu.Status|cpu.flags.B|cpu.flags.U)
	cpu.setFlag(cpu.flags.B, false)
	cpu.setFlag(cpu.flags.U, false)
	cpu.Sp--

	return 0
}

func (cpu *CPU6502) pla() uint8 {
	cpu.Sp++
	cpu.A = cpu.Read(0x0100 + uint16(cpu.Sp))

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 0
}

func (cpu *CPU6502) plp() uint8 {
	cpu.Sp++
	cpu.Status = cpu.Read(0x0100 + uint16(cpu.Sp))
	cpu.setFlag(cpu.flags.U, true)

	return 0
}

func (cpu *CPU6502) rol() uint8 {
	cpu.fetch()

	tmp := (uint16(cpu.fetched) << 1) | uint16(cpu.getFlag(cpu.flags.C))

	cpu.setFlag(cpu.flags.C, tmp&0xFF00 > 0)
	cpu.setFlag(cpu.flags.Z, tmp&0x00FF == 0x0000)
	cpu.setFlag(cpu.flags.N, tmp&0x00FF == 0x0080)

	if cpu.isImp {
		cpu.A = uint8(tmp & 0x00FF)
	} else {
		cpu.Write(cpu.addrAbs, uint8(tmp&0x00FF))
	}

	return 0
}

func (cpu *CPU6502) ror() uint8 {
	cpu.fetch()

	tmp := uint16(cpu.getFlag(cpu.flags.C)<<7) | uint16(cpu.fetched>>1)

	cpu.setFlag(cpu.flags.C, cpu.fetched&0x01 > 0)
	cpu.setFlag(cpu.flags.Z, tmp&0x00FF == 0x0000)
	cpu.setFlag(cpu.flags.N, tmp&0x0080 > 0)

	if cpu.isImp {
		cpu.A = uint8(tmp & 0x00FF)
	} else {
		cpu.Write(cpu.addrAbs, uint8(tmp&0x00FF))
	}

	return 0
}

func (cpu *CPU6502) rti() uint8 {
	// Pull status register from stack and ignore B and U flags
	cpu.Sp++
	cpu.Status = cpu.Read(0x0100 + uint16(cpu.Sp))
	cpu.Status &= ^cpu.flags.B
	cpu.Status &= ^cpu.flags.U

	// Pull program counter from stack
	cpu.Sp++
	lb := uint16(cpu.Read(0x0100 + uint16(cpu.Sp)))

	cpu.Sp++
	hb := uint16(cpu.Read(0x0100 + uint16(cpu.Sp)))

	cpu.Pc = (hb << 8) | lb

	return 0
}

func (cpu *CPU6502) rts() uint8 {
	cpu.Sp++
	lb := uint16(cpu.Read(0x0100 + uint16(cpu.Sp)))

	cpu.Sp++
	hb := uint16(cpu.Read(0x0100 + uint16(cpu.Sp)))

	cpu.Pc = (hb << 8) | lb
	cpu.Pc++

	return 0
}

func (cpu *CPU6502) sbc() uint8 {
	cpu.fetch()

	data := uint16(cpu.fetched) ^ 0x00FF

	tmp := uint16(cpu.A) + data + uint16(cpu.getFlag(cpu.flags.C))
	cpu.setFlag(cpu.flags.C, (tmp&0xFF00) > 0)
	cpu.setFlag(cpu.flags.Z, (tmp&0x00FF) == 0)
	cpu.setFlag(cpu.flags.N, (tmp&0x80) > 0)

	cpu.setFlag(
		cpu.flags.V,
		((^(uint16(cpu.A)^data)&(uint16(cpu.A)^tmp))&0x0080) > 0,
	)

	cpu.A = uint8(tmp & 0x00FF)

	return 0
}

func (cpu *CPU6502) sec() uint8 {
	cpu.setFlag(cpu.flags.C, true)
	return 0
}

func (cpu *CPU6502) sed() uint8 {
	cpu.setFlag(cpu.flags.D, true)
	return 0
}

func (cpu *CPU6502) sei() uint8 {
	cpu.setFlag(cpu.flags.I, true)
	return 0
}

func (cpu *CPU6502) sta() uint8 {
	cpu.Write(cpu.addrAbs, cpu.A)
	return 0
}

func (cpu *CPU6502) stx() uint8 {
	cpu.Write(cpu.addrAbs, cpu.X)
	return 0
}

func (cpu *CPU6502) sty() uint8 {
	cpu.Write(cpu.addrAbs, cpu.Y)
	return 0
}

func (cpu *CPU6502) tax() uint8 {
	cpu.X = cpu.A

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 0
}

func (cpu *CPU6502) tay() uint8 {
	cpu.Y = cpu.A

	cpu.setFlag(cpu.flags.Z, cpu.Y == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.Y&0x80 > 0)

	return 0
}

func (cpu *CPU6502) tsx() uint8 {
	cpu.X = cpu.Sp

	cpu.setFlag(cpu.flags.Z, cpu.X == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.X&0x80 > 0)

	return 0
}

func (cpu *CPU6502) txa() uint8 {
	cpu.A = cpu.X

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 0
}

func (cpu *CPU6502) txs() uint8 {
	cpu.Sp = cpu.X

	return 0
}

func (cpu *CPU6502) tya() uint8 {
	cpu.A = cpu.Y

	cpu.setFlag(cpu.flags.Z, cpu.A == 0x00)
	cpu.setFlag(cpu.flags.N, cpu.A&0x80 > 0)

	return 0
}

// Used for illegal addresses
func (cpu *CPU6502) xxx() uint8 {
	return 0
}

// ============================================
