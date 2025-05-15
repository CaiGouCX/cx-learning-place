import re

# Map for register to binary
reg_map = {f'x{i}': format(i, '05b') for i in range(32)}

# Opcodes and funct codes for RV32I 40 instructions
opcodes = {
    'add':  ('0110011', '000', '0000000'),
    'sub':  ('0110011', '000', '0100000'),
    'sll':  ('0110011', '001', '0000000'),
    'slt':  ('0110011', '010', '0000000'),
    'sltu': ('0110011', '011', '0000000'),
    'xor':  ('0110011', '100', '0000000'),
    'srl':  ('0110011', '101', '0000000'),
    'sra':  ('0110011', '101', '0100000'),
    'or':   ('0110011', '110', '0000000'),
    'and':  ('0110011', '111', '0000000'),

    'addi': ('0010011', '000', None),
    'slti': ('0010011', '010', None),
    'sltiu':('0010011', '011', None),
    'xori': ('0010011', '100', None),
    'ori':  ('0010011', '110', None),
    'andi': ('0010011', '111', None),
    'slli': ('0010011', '001', '0000000'),
    'srli': ('0010011', '101', '0000000'),
    'srai': ('0010011', '101', '0100000'),

    'lb':   ('0000011', '000', None),
    'lh':   ('0000011', '001', None),
    'lw':   ('0000011', '010', None),
    'lbu':  ('0000011', '100', None),
    'lhu':  ('0000011', '101', None),

    'sb':   ('0100011', '000', None),
    'sh':   ('0100011', '001', None),
    'sw':   ('0100011', '010', None),

    'beq':  ('1100011', '000', None),
    'bne':  ('1100011', '001', None),
    'blt':  ('1100011', '100', None),
    'bge':  ('1100011', '101', None),
    'bltu': ('1100011', '110', None),
    'bgeu': ('1100011', '111', None),

    'lui':  ('0110111', None, None),
    'auipc':('0010111', None, None),
    'jal':  ('1101111', None, None),
    'jalr': ('1100111', '000', None),
}
def format_binary_instruction(binary_str):
    assert len(binary_str) == 32, "必须是32位的二进制字符串"
    
    # 每8位分段
    bytes_ = [binary_str[i:i+8] for i in range(0, 32, 8)]

    print("Byte\tBinary\t\tHex\tDec")
    print("-" * 40)
    for i, byte in enumerate(bytes_):
        hex_val = hex(int(byte, 2))[2:].zfill(2)
        dec_val = int(byte, 2)
        print(f"B{i}\t{byte}\t0x{hex_val}\t{dec_val}")




def reg_bin(reg):
    if reg not in reg_map:
        raise ValueError(f"Unknown register: {reg}")
    return reg_map[reg]

def imm_bin(val, bits):
    return format(val & ((1 << bits) - 1), f'0{bits}b')

def parse_mem_operand(operand):
    match = re.match(r'(-?\d+)\((x\d+)\)', operand.replace(' ', ''))
    if not match:
        raise ValueError(f"Invalid memory operand format: {operand}")
    return int(match.group(1)), match.group(2)

def assemble(instr):
    parts = instr.replace(',', '').split()
    name = parts[0]
    if name not in opcodes:
        raise ValueError("Unsupported instruction")
    opc, funct3, funct7 = opcodes[name]

    if opc == '0110011':  # R-type
        rd, rs1, rs2 = reg_bin(parts[1]), reg_bin(parts[2]), reg_bin(parts[3])
        return funct7 + rs2 + rs1 + funct3 + rd + opc

    elif opc == '0010011':  # I-type arith/shift
        rd, rs1 = reg_bin(parts[1]), reg_bin(parts[2])
        imm = int(parts[3],0)
        if name in ['slli', 'srli', 'srai']:
            return funct7 + imm_bin(imm, 5) + rs1 + funct3 + rd + opc
        else:
            return imm_bin(imm, 12) + rs1 + funct3 + rd + opc

    elif opc == '0000011':  # Load
        rd = reg_bin(parts[1])
        imm, rs1_name = parse_mem_operand(parts[2])
        rs1 = reg_bin(rs1_name)
        return imm_bin(imm, 12) + rs1 + funct3 + rd + opc

    elif opc == '0100011':  # Store
        rs2 = reg_bin(parts[1])
        imm, rs1_name = parse_mem_operand(parts[2])
        rs1 = reg_bin(rs1_name)
        imm12 = imm_bin(imm, 12)
        return imm12[:7] + rs2 + rs1 + funct3 + imm12[7:] + opc

    elif opc == '1100011':  # Branch
        rs1, rs2 = reg_bin(parts[1]), reg_bin(parts[2])
        imm = int(parts[3])
        imm_bin_str = imm_bin(imm >> 1, 12)
        return (imm_bin_str[0] + imm_bin_str[2:8] + rs2 + rs1 + funct3 +
                imm_bin_str[8:12] + imm_bin_str[1] + opc)

    elif opc == '0110111' or opc == '0010111':  # U-type
        rd = reg_bin(parts[1])
        imm = int(parts[2],0)
        return imm_bin(imm, 20) + rd + opc

    elif opc == '1101111':  # J-type jal
        rd = reg_bin(parts[1])
        imm = int(parts[2],0) >> 1
        imm_bin_str = imm_bin(imm, 20)
        return (imm_bin_str[0] + imm_bin_str[10:20] + imm_bin_str[9] +
                imm_bin_str[1:9] + rd + opc)

    elif opc == '1100111':  # jalr
        rd = reg_bin(parts[1])
        imm, rs1_name = parse_mem_operand(parts[2])
        rs1 = reg_bin(rs1_name)
        return imm_bin(imm, 12) + rs1 + funct3 + rd + opc

    else:
        raise ValueError("Unsupported instruction format")
    
def binstr_to_little_endian_dec(binstr):
    assert len(binstr) == 32
    val = int(binstr, 2)
    bytes_le = [(val >> (8 * i)) & 0xFF for i in range(4)]
    return ' '.join(str(b) for b in bytes_le)

# Example usage:
instruction = [
    "lui x1, 0x12345" ,          # x1 = 0x12345000
    "auipc x2, 0x1",            # x2 = PC + 0x10000

    "addi x3, x1, 5  ",          # x3 = x1 + 5
    "slti x4, x3, 10  ",         # x4 = x3 < 10 ? 1 : 0
    "sltiu x5, x3, 10 ",         # x5 = x3 < 10 (unsigned)

    "xori x6, x3, 0xFF ",        # x6 = x3 ^ 0xFF
    "ori x7, x3, 0x0F ",         # x7 = x3 | 0x0F
    "andi x8, x3, 0xF0 ",        # x8 = x3 & 0xF0

    "slli x9, x3, 2 " ,          # x9 = x3 << 2
    "srli x10, x3, 1 " ,         # x10 = logical x3 >> 1
    "srai x11, x3, 1 " ,         # x11 = arithmetic x3 >> 1

    "add x12, x3, x2 " ,         # x12 = x3 + x2
    "sub x13, x12, x3 " ,        # x13 = x12 - x3
    "sll x14, x3, x4  " ,        # x14 = x3 << (x4 % 32)
    "slt x15, x4, x3   ",        # x15 = x4 < x3
    "sltu x16, x4, x3 " ,        # x16 = x4 < x3 (unsigned)
    "xor x17, x4, x5  " ,        # x17 = x4 ^ x5
    "srl x18, x3, x4   " ,       # x18 = x3 >> (x4 % 32)
    "sra x19, x3, x4 " ,         # x19 = arithmetic shift
    "or x20, x4, x5  " ,         # x20 = x4 | x5
    "and x21, x4, x5 " ,         # x21 = x4 & x5

    "sb x3, 0(x0)" ,             # store byte
    "sh x3, 2(x0) ",             # store half
    "sw x3, 4(x0)" ,             # store word

    "lb x22, 0(x0)  " ,          # load byte
    "lh x23, 2(x0) " ,           # load half
    "lw x24, 4(x0) " ,           # load word
    "lbu x25, 0(x0) "  ,         # load byte unsigned
    "lhu x26, 2(x0) " ,          # load half unsigned

    "beq x4, x4, 0" ,       # branch equal
    "bne x4, x5, 0"  ,      # branch not equal
    "blt x4, x5, 0 "  ,     # branch less than
    "bge x5, x4, 0 "   ,    # branch greater/equal
    "bltu x4, x5, 0  " ,    # unsigned branch less than
    "bgeu x5, x4, 0 " ,     # unsigned branch greater/equal

    "jal x2, 32  "  ,       # jump and link
    "jalr x2, 0(x1) " ,         # jump and link register

    "addi x0, x0, 0 "           # no-op (label target)

]

for ins in instruction:
    print(ins)
    print(assemble(ins))
    format_binary_instruction(assemble(ins))

for ins in instruction:
    bin_code = assemble(ins)  # 你的assemble函数需返回32位二进制字符串
    dec_bytes = binstr_to_little_endian_dec(bin_code)
    print(dec_bytes)
#print(assemble("ecall"))
#print(assemble("ebreak"))


