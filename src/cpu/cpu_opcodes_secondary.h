        case 0xE0: // CPX Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->x - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->x >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xE4: // CPX Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->x - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->x >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xEC: // CPX Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
                uint8_t result = cpu->x - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->x >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC0: // CPY Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC4: // CPY Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCC: // CPY Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // BIT tests.
        case 0x24: // BIT Zero Page
            {
                uint8_t val = read_mem(get_zpg_address(cpu));
                cpu->status = (cpu->status & ~(ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a & val) == 0 ? ZERO_FLAG : 0) |
                             (val & 0x40 ? OVERFLOW_FLAG : 0) |
                             (val & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0x2C: // BIT Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                cpu->status = (cpu->status & ~(ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a & val) == 0 ? ZERO_FLAG : 0) |
                             (val & 0x40 ? OVERFLOW_FLAG : 0) |
                             (val & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // Left shifts.
        case 0x0A: // ASL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ASL A
            asl(cpu, &cpu->a);
            break;
        case 0x06: // ASL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x16: // ASL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x0E: // ASL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x1E: // ASL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Right shifts.
        case 0x4A: // LSR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte LSR A
            lsr(cpu, &cpu->a);
            break;
        case 0x46: // LSR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x56: // LSR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x4E: // LSR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x5E: // LSR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Left rotates.
        case 0x2A: // ROL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROL A
            rol(cpu, &cpu->a);
            break;
        case 0x26: // ROL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x36: // ROL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x2E: // ROL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x3E: // ROL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Right rotates.
        case 0x6A: // ROR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROR A
            ror(cpu, &cpu->a);
            break;
        case 0x66: // ROR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x76: // ROR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x6E: // ROR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x7E: // ROR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;

        // Memory increments.
        case 0xE6: // INC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xF6: // INC Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xEE: // INC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xFE: // INC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // Memory decrements.
        case 0xC6: // DEC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xD6: // DEC Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xCE: // DEC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xDE: // DEC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // Undocumented NOP variants.
        // NOP Zero Page
        case 0x04: case 0x44: case 0x64:
            {
                uint8_t zpg = read_mem(cpu->pc++);
                (void)read_mem(zpg);
            }
            break;
        // NOP Zero Page,X
        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            {
                uint8_t zpg = read_mem(cpu->pc++);
                (void)read_mem(zpg); // indexed addressing dummy read
                (void)read_mem((uint8_t)(zpg + cpu->x));
            }
            break;
        // NOP Absolute
        case 0x0C:
            {
                uint16_t addr = get_abs_address(cpu);
                (void)read_mem(addr);
            }
            break;
        // NOP Absolute,X
        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            (void)read_mem(get_absx_read(cpu));
            break;
        // Implied NOPs (1 byte) - Add dummy reads
        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:
            dummy_read_next(cpu);  // Add dummy read for undocumented implied NOPs
            break;
        // JAM: neither interrupts nor later instructions can restart the CPU.
        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
            dummy_read_next(cpu);
            cpu->pc--;
            cpu->halted = true;
            cpu_nmi_ready = cpu_irq_ready = false;
            break;
        // Immediate NOPs (consume operand)
        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            (void)read_mem(cpu->pc++);
            break;

        // DCP: decrement memory, then compare with the accumulator.
        case 0xC7: // DCP Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD7: // DCP Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCF: // DCP Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDF: // DCP Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDB: // DCP Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC3: // DCP (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD3: // DCP (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // RLA: rotate memory left, then AND it with the accumulator.
        case 0x27: // RLA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x37: // RLA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x2F: // RLA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3F: // RLA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3B: // RLA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x23: // RLA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x33: // RLA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // SLO: shift memory left, then OR it with the accumulator.
        case 0x07: // SLO Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x17: // SLO Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x0F: // SLO Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1F: // SLO Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1B: // SLO Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x03: // SLO (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x13: // SLO (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // SRE: shift memory right, then XOR it with the accumulator.
        case 0x47: // SRE Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x57: // SRE Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x4F: // SRE Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5F: // SRE Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5B: // SRE Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x43: // SRE (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x53: // SRE (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // RRA: rotate memory right, then add it to the accumulator.
        case 0x67: // RRA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x77: // RRA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x6F: // RRA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7F: // RRA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7B: // RRA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x63: // RRA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x73: // RRA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;

        // LAX loads the same value into A and X.
        case 0xA7: // LAX Zero Page
            {
                uint8_t val = read_mem(get_zpg_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xB7: // LAX Zero Page,Y
            {
                uint8_t val = read_mem(get_zpgy_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xAF: // LAX Absolute
            {
                uint8_t val = read_mem(get_abs_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xBF: // LAX Absolute,Y
            {
                uint8_t val = read_mem(get_absy_read(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xA3: // LAX (indirect,X)
            {
                uint8_t val = read_mem(get_indirect_x(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xB3: // LAX (indirect),Y
            {
                uint8_t val = read_mem(get_indirect_y_read(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xAB: // LAX Immediate (unofficial)
            {
                uint8_t val = read_mem(cpu->pc++);
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;

        // SAX stores A AND X.
        case 0x87: // SAX Zero Page
            write_mem(get_zpg_address(cpu), cpu->a & cpu->x);
            break;
        case 0x97: // SAX Zero Page,Y
            write_mem(get_zpgy_address(cpu), cpu->a & cpu->x);
            break;
        case 0x8F: // SAX Absolute
            write_mem(get_abs_address(cpu), cpu->a & cpu->x);
            break;
        case 0x83: // SAX (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a & cpu->x);
            break;

        // ISC increments memory, then subtracts it from the accumulator.
        case 0xE7: // ISC Zero Page
            inc_then_sbc(cpu, get_zpg_address(cpu));
            break;
        case 0xF7: // ISC Zero Page,X
            inc_then_sbc(cpu, get_zpgx_address(cpu));
            break;
        case 0xEF: // ISC Absolute
            inc_then_sbc(cpu, get_abs_address(cpu));
            break;
        case 0xFF: // ISC Absolute,X
            inc_then_sbc(cpu, get_absx_address(cpu));
            break;
        case 0xFB: // ISC Absolute,Y
            inc_then_sbc(cpu, get_absy_address(cpu));
            break;
        case 0xE3: // ISC (indirect,X)
            inc_then_sbc(cpu, get_indirect_x(cpu));
            break;
        case 0xF3: // ISC (indirect),Y
            inc_then_sbc(cpu, get_indirect_y(cpu));
            break;

        // LAS loads A, X, and the stack pointer from the same masked value.
        case 0xBB: // LAS Absolute,Y
            {
                uint8_t mem = read_mem(get_absy_read(cpu));
                uint8_t result = mem & cpu->sp;
                cpu->a = cpu->x = cpu->sp = result;
                set_zn_flags(cpu, result);
            }
            break;

        // Unstable SHX, SHY, SHA, and TAS stores.
        case 0x9F: // AHX/SHA Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->a & cpu->x);
            break;
        case 0x93: // AHX/SHA (indirect),Y
            {
                uint8_t zpg = read_mem(cpu->pc++);
                masked_indexed_store(read_zpg_word(zpg), cpu->y, cpu->a & cpu->x);
            }
            break;
        case 0x9E: // SHX Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->x);
            break;
        case 0x9C: // SHY Absolute,X
            masked_indexed_store(get_abs_address(cpu), cpu->x, cpu->y);
            break;
        case 0x9B: // TAS/SHS Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->a & cpu->x);
            cpu->sp = cpu->a & cpu->x;
            break;

