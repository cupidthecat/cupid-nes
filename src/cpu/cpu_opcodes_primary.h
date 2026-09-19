/*
 * cpu_opcodes_primary.h - Primary CPU opcode handlers
 *
 * Author: @frankischilling
 *
 * This private header contains the first group of opcode handlers used by the CPU instruction dispatcher.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef CPU_OPCODES_PRIMARY_H
#define CPU_OPCODES_PRIMARY_H

        // Control flow.
        case 0x00: { // BRK
            // 1) Dummy read of the signature byte *and* advance PC (BRK behaves like 2-byte)
            (void)read_mem(cpu->pc++);
        
            // 2) Push return address = PC after the dummy read (i.e., address of next instruction)
            uint16_t ret = cpu->pc;
            write_mem(0x0100 + cpu->sp--, (ret >> 8) & 0xFF);
            write_mem(0x0100 + cpu->sp--, ret & 0xFF);
            // An NMI detected during the stack pushes can replace BRK's vector.
            bool nmi_interrupts_brk = cpu_nmi_pending;
        
            // 3) Push status with B=1, U=1
            uint8_t p = (cpu->status | BREAK_FLAG | UNUSED_FLAG);
            write_mem(0x0100 + cpu->sp--, p);
        
            // 4) Set I
            cpu->status |= INTERRUPT_FLAG;
        
            // 5) Load vector (NMI has priority if it interrupts BRK)
            if (nmi_interrupts_brk) {
                cpu_nmi_pending = false;
                cpu_nmi_injected = false;
                NMI_LOG("BRK sequence interrupted by NMI");
                cpu->pc = read_mem_word(0xFFFA);
                NMI_BRK_LOG("BRK->NMI vector=%04X pushedP=%02X", cpu->pc, p);
            } else {
                cpu->pc = read_mem_word(0xFFFE);
                NMI_BRK_LOG("BRK->IRQ vector=%04X pushedP=%02X", cpu->pc, p);
            }
            // An edge detected during the vector fetch waits for the handler's first instruction.
            cpu_nmi_ready = false;
        } break;        
        case 0xEA: // NOP
            dummy_read_next(cpu);  // Add dummy read for 1-byte NOP
            break;
        case 0x4C: // JMP Absolute
            {
                uint16_t addr = read_mem(cpu->pc);
                cpu->pc++;
                addr |= read_mem(cpu->pc) << 8;
                cpu->pc++;
                cpu->pc = addr;
            }
            break;
        case 0x6C: // JMP Indirect
            cpu->pc = get_abs_indirect(cpu);
            break;

        // Subroutines.
        case JSR_OPCODE: // 0x20
        {
            // 6502 JSR bus order:
            // 2) read target low from PC, increment PC
            // 3) dummy read from stack
            // 4) push PCH of return address
            // 5) push PCL of return address
            // 6) read target high from PC and jump
            uint8_t low = read_mem(cpu->pc++);
            (void)read_mem((uint16_t)(0x0100u + cpu->sp));

            write_mem((uint16_t)(0x0100u + cpu->sp), (uint8_t)((cpu->pc >> 8) & 0xFFu));
            cpu->sp--;
            write_mem((uint16_t)(0x0100u + cpu->sp), (uint8_t)(cpu->pc & 0xFFu));
            cpu->sp--;

            uint8_t high = read_mem(cpu->pc);
            cpu->pc = (uint16_t)(((uint16_t)high << 8) | low);
        }
        break;
        
        // FIXED: RTS - Return from Subroutine
        case RTS_OPCODE: // 0x60
        {
            dummy_read_next(cpu);  // Required dummy fetch
            (void)read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t low  = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t high = read_mem(0x0100 + cpu->sp);
            uint16_t return_addr = (high << 8) | low;
            (void)read_mem(return_addr);
            
            // Return address on stack is the last byte of JSR, so add 1 to skip past it
            cpu->pc = return_addr + 1;
        }
        break;
        case 0x40: // RTI - Return from Interrupt
        {
            dummy_read_next(cpu);              // RTI dummy read
            (void)read_mem(0x0100 + cpu->sp);
            // Pull P (SP increments before each pull)
            cpu->sp++;
            uint8_t newP = read_mem(0x0100 + cpu->sp);
            cpu->status = (newP & ~BREAK_FLAG) | UNUSED_FLAG;

            // RTI restores the saved PC directly; unlike RTS, it does not add one.
            cpu->sp++;
            uint8_t pcl = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t pch = read_mem(0x0100 + cpu->sp);
            cpu->pc = ((uint16_t)pch << 8) | pcl;
            NMI_BRK_LOG("RTI to %04X newP=%02X", cpu->pc, cpu->status);
        }
        break;

        // Stack operations.
        case 0x48: // PHA - Push Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PHA
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;
            break;
        case 0x68: // PLA - Pull Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PLA
            (void)read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            cpu->a = read_mem(0x0100 + cpu->sp);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x08: // PHP - Push Processor Status
            dummy_read_next(cpu);  // Add dummy read for 1-byte PHP
            write_mem(0x0100 + cpu->sp, cpu->status | BREAK_FLAG | UNUSED_FLAG);
            cpu->sp--;
            break;
        case 0x28: // PLP - Pull Processor Status
            dummy_read_next(cpu);  // Add dummy read for 1-byte PLP
            (void)read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            {
                uint8_t newP = read_mem(0x0100 + cpu->sp);
                cpu->status = (newP & ~BREAK_FLAG) | UNUSED_FLAG;
            }
            break;

        // Branches.
        case 0x90: // BCC - Branch if Carry Clear
            branch_if(cpu, !(cpu->status & CARRY_FLAG));
            break;
        case 0xB0: // BCS - Branch if Carry Set
            branch_if(cpu, (cpu->status & CARRY_FLAG));
            break;
        case 0xF0: // BEQ - Branch if Equal (Zero)
            branch_if(cpu, (cpu->status & ZERO_FLAG));
            break;
        case 0xD0: // BNE - Branch if Not Equal
            branch_if(cpu, !(cpu->status & ZERO_FLAG));
            break;
        case 0x30: // BMI - Branch if Minus (Negative)
            branch_if(cpu, cpu->status & NEGATIVE_FLAG);
            break;
        case 0x10: // BPL - Branch if Plus
            branch_if(cpu, !(cpu->status & NEGATIVE_FLAG));
            break;
        case 0x70: // BVS - Branch if Overflow Set
            branch_if(cpu, cpu->status & OVERFLOW_FLAG);
            break;
        case 0x50: // BVC - Branch if Overflow Clear
            branch_if(cpu, !(cpu->status & OVERFLOW_FLAG));
            break;

        // Status flags.
        case 0x18: // CLC - Clear Carry Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLC
            cpu->status &= ~CARRY_FLAG;
            break;
        case 0x38: // SEC - Set Carry Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SEC
            cpu->status |= CARRY_FLAG;
            break;
        case 0xD8: // CLD - Clear Decimal Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLD
            cpu->status &= ~DECIMAL_FLAG;
            break;
        case 0xF8: // SED - Set Decimal Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SED
            cpu->status |= DECIMAL_FLAG;
            break;
        case 0x58: // CLI - Clear Interrupt Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLI
            cpu->status &= ~INTERRUPT_FLAG;
            break;
        case 0x78: // SEI - Set Interrupt Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SEI
            cpu->status |= INTERRUPT_FLAG;
            break;
        case 0xB8: // CLV - Clear Overflow Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLV
            cpu->status &= ~OVERFLOW_FLAG;
            break;

        // Accumulator loads.
        case 0xA9: // LDA Immediate
            cpu->a = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xA5: // LDA Zero Page
            cpu->a = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xB5: // LDA Zero Page,X
            cpu->a = read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xAD: // LDA Absolute
            cpu->a = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xBD: // LDA Absolute,X
            cpu->a = read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xB9: // LDA Absolute,Y
            cpu->a = read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xA1: // LDA (indirect,X)
            cpu->a = read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xB1: // LDA (indirect),Y
            cpu->a = read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // Accumulator stores.
        case 0x85: // STA Zero Page
            write_mem(get_zpg_address(cpu), cpu->a);
            break;
        case 0x95: // STA Zero Page,X
            write_mem(get_zpgx_address(cpu), cpu->a);
            break;
        case 0x8D: // STA Absolute
            write_mem(get_abs_address(cpu), cpu->a);
            break;
        case 0x9D: { // STA abs,X
            uint16_t a = get_absx_address(cpu);
            write_mem(a, cpu->a);
        } break;
        case 0x99: { // STA abs,Y
            uint16_t a = get_absy_address(cpu);
            write_mem(a, cpu->a);
        } break;   
        case 0x81: // STA (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a);
            break;
        case 0x91: { // STA (ind),Y
            uint16_t a = get_indirect_y(cpu);
            write_mem(a, cpu->a);
        } break;

        // X-register loads.
        case 0xA2: // LDX Immediate
            cpu->x = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xA6: // LDX Zero Page
            cpu->x = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xB6: // LDX Zero Page,Y
            cpu->x = read_mem(get_zpgy_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xAE: // LDX Absolute
            cpu->x = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xBE: // LDX Absolute,Y
            cpu->x = read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        // X-register stores.
        case 0x86: // STX Zero Page
            write_mem(get_zpg_address(cpu), cpu->x);
            break;
        case 0x96: // STX Zero Page,Y
            write_mem(get_zpgy_address(cpu), cpu->x);
            break;
        case 0x8E: // STX Absolute
            write_mem(get_abs_address(cpu), cpu->x);
            break;

        // Y-register loads.
        case 0xA0: // LDY Immediate
            cpu->y = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xA4: // LDY Zero Page
            cpu->y = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xB4: // LDY Zero Page,X
            cpu->y = read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xAC: // LDY Absolute
            cpu->y = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xBC: // LDY Absolute,X
            cpu->y = read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->y);
            break;

        // Y-register stores.
        case 0x84: // STY Zero Page
            write_mem(get_zpg_address(cpu), cpu->y);
            break;
        case 0x94: // STY Zero Page,X
            write_mem(get_zpgx_address(cpu), cpu->y);
            break;
        case 0x8C: // STY Absolute
            write_mem(get_abs_address(cpu), cpu->y);
            break;

        // Register transfers.
        case 0xAA: // TAX - Transfer A to X
            dummy_read_next(cpu);  // Add dummy read for 1-byte TAX
            cpu->x = cpu->a;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0x8A: // TXA - Transfer X to A
            dummy_read_next(cpu);  // Add dummy read for 1-byte TXA
            cpu->a = cpu->x;
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xA8: // TAY - Transfer A to Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte TAY
            cpu->y = cpu->a;
            set_zn_flags(cpu, cpu->y);
            break;
        case 0x98: // TYA - Transfer Y to A
            dummy_read_next(cpu);  // Add dummy read for 1-byte TYA
            cpu->a = cpu->y;
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xBA: // TSX - Transfer SP to X
            dummy_read_next(cpu);  // Add dummy read for 1-byte TSX
            cpu->x = cpu->sp;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0x9A: // TXS - Transfer X to SP
            dummy_read_next(cpu);  // Add dummy read for 1-byte TXS
            cpu->sp = cpu->x;
            break;

        // Register increments and decrements.
        case 0xE8: // INX - Increment X
            dummy_read_next(cpu);  // Add dummy read for 1-byte INX
            cpu->x++;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xCA: // DEX - Decrement X
            dummy_read_next(cpu);  // Add dummy read for 1-byte DEX
            cpu->x--;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xC8: // INY - Increment Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte INY
            cpu->y++;
            set_zn_flags(cpu, cpu->y);
            break;
        case 0x88: // DEY - Decrement Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte DEY
            cpu->y--;
            set_zn_flags(cpu, cpu->y);
            break;

        // AND operations.
        case 0x29: // AND Immediate
            cpu->a &= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x25: // AND Zero Page
            cpu->a &= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x35: // AND Zero Page,X
            cpu->a &= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x2D: // AND Absolute
            cpu->a &= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x3D: // AND Absolute,X
            cpu->a &= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x39: // AND Absolute,Y
            cpu->a &= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x21: // AND (indirect,X)
            cpu->a &= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x31: // AND (indirect),Y
            cpu->a &= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // ANC: AND the operand, then copy the negative flag into carry.
        case 0x0B: // ANC Immediate
        case 0x2B: // ANC Immediate (alternate opcode)
            cpu->a &= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            if (cpu->a & 0x80) cpu->status |= CARRY_FLAG;
            else               cpu->status &= ~CARRY_FLAG;
            break;

        // ALR: AND the operand, then shift the accumulator right.
        case 0x4B: // ALR Immediate
            cpu->a &= read_mem(cpu->pc++);
            lsr(cpu, &cpu->a);
            break;

        case 0x8B: { // XAA/ANE Immediate
            // Silicon-dependent analog behavior is modeled with a fixed $EE mask.
            uint8_t imm = read_mem(cpu->pc++);
            cpu->a = (uint8_t)((cpu->a | 0xEEu) & cpu->x & imm);
            set_zn_flags(cpu, cpu->a);
            break;
        }

        // ARR: AND the operand, rotate right, then apply its unusual flag rules.
        case 0x6B: // ARR Immediate
            {
                uint8_t imm = read_mem(cpu->pc++);
                uint8_t carry_in = (cpu->status & CARRY_FLAG) ? 1 : 0;
                uint8_t tmp = cpu->a & imm;
                uint8_t result = (tmp >> 1) | (carry_in << 7);

                cpu->a = result;
                set_zn_flags(cpu, cpu->a);

                if (result & 0x40) cpu->status |= CARRY_FLAG;
                else               cpu->status &= ~CARRY_FLAG;

                if (((result >> 6) ^ (result >> 5)) & 0x01) cpu->status |= OVERFLOW_FLAG;
                else                                        cpu->status &= ~OVERFLOW_FLAG;
            }
            break;

        // AXS/SBX undocumented opcode.
        case 0xCB: // AXS/SBX Immediate: X = (A & X) - imm
            {
                uint8_t imm = read_mem(cpu->pc++);
                uint8_t and_ax = cpu->a & cpu->x;
                uint8_t result = (uint8_t)(and_ax - imm);

                if (and_ax >= imm) cpu->status |= CARRY_FLAG;
                else               cpu->status &= ~CARRY_FLAG;

                cpu->x = result;
                set_zn_flags(cpu, cpu->x);
            }
            break;

        // ORA operations.
        case 0x09: // ORA Immediate
            cpu->a |= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x05: // ORA Zero Page
            cpu->a |= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x15: // ORA Zero Page,X
            cpu->a |= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x0D: // ORA Absolute
            cpu->a |= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x1D: // ORA Absolute,X
            cpu->a |= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x19: // ORA Absolute,Y
            cpu->a |= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x01: // ORA (indirect,X)
            cpu->a |= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x11: // ORA (indirect),Y
            cpu->a |= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // EOR operations.
        case 0x49: // EOR Immediate
            cpu->a ^= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x45: // EOR Zero Page
            cpu->a ^= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x55: // EOR Zero Page,X
            cpu->a ^= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x4D: // EOR Absolute
            cpu->a ^= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x5D: // EOR Absolute,X
            cpu->a ^= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x59: // EOR Absolute,Y
            cpu->a ^= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x41: // EOR (indirect,X)
            cpu->a ^= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x51: // EOR (indirect),Y
            cpu->a ^= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // Addition.
        case 0x69: // ADC Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;
        case 0x65: 
            do_adc(cpu, read_mem(get_zpg_address(cpu)));  
            break;
        case 0x75: 
            do_adc(cpu, read_mem(get_zpgx_address(cpu))); 
            break;
        case 0x6D: 
            do_adc(cpu, read_mem(get_abs_address(cpu)));  
            break;
        case 0x7D: 
            do_adc(cpu, read_mem(get_absx_read(cpu)));
            break;
        case 0x79: 
            do_adc(cpu, read_mem(get_absy_read(cpu)));
            break;
        case 0x61: // ADC (indirect,X)
            {
                uint8_t operand = read_mem(get_indirect_x(cpu));
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;
        case 0x71: // ADC (indirect),Y
            {
                uint8_t operand = read_mem(get_indirect_y_read(cpu));
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;

        // Subtraction.
        case 0xE9: // SBC Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                            (diff < 0x100 ? CARRY_FLAG : 0) |
                            ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                            (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                            (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;
        case 0xEB: // SBC Immediate (Unofficial)
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                            (diff < 0x100 ? CARRY_FLAG : 0) |
                            ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                            (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                            (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;
        case 0xE5: 
            do_sbc(cpu, read_mem(get_zpg_address(cpu)));  
            break;
        case 0xF5: 
            do_sbc(cpu, read_mem(get_zpgx_address(cpu))); 
            break;
        case 0xED: 
            do_sbc(cpu, read_mem(get_abs_address(cpu)));  
            break;
        case 0xFD: 
            do_sbc(cpu, read_mem(get_absx_read(cpu)));
            break;
        case 0xF9: 
            do_sbc(cpu, read_mem(get_absy_read(cpu))); 
            break;

        case 0xE1: // SBC (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t operand = read_mem(addr);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (diff < 0x100 ? CARRY_FLAG : 0) |
                             ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;
        case 0xF1:
            do_sbc(cpu, read_mem(get_indirect_y_read(cpu)));
            break;

        // Comparisons.
        case 0xC9: // CMP Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

#endif // CPU_OPCODES_PRIMARY_H
        case 0xC5: // CMP Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD5: // CMP Zero Page,X
            {
                uint8_t operand = read_mem(get_zpgx_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCD: // CMP Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDD: // CMP Absolute,X
            {
                uint8_t operand = read_mem(get_absx_read(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD9: // CMP Absolute,Y
            {
                uint8_t operand = read_mem(get_absy_read(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC1: // CMP (indirect,X)
            {
                uint8_t operand = read_mem(get_indirect_x(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD1: // CMP (indirect),Y
            {
                uint8_t operand = read_mem(get_indirect_y_read(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

