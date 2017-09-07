/****************************************************************
 * Author: Eric Mackay
 * Date: August 9, 2017
 * Project: TinyEmulator - An Emulator for TinyCPU
 * File: emulator.h
 ****************************************************************/

#ifndef EMULATOR_H
#define EMULATOR_H

#include <stdbool.h>
#include <stdint.h>

/****************************************************************
 * Tiny CPU is Big Endian
 *
 * Instruction Format
 * Instructions are 2 bytes long
 * Opcode is always bits 15:12
 *
 * Memory has 1 byte addresses for a total of 2^8=256 locations
 * Memory addressability is 1 byte, i.e., 1 byte per memory location
 * 
 * There are 8 general registers: r0-r7
 * Registers store 1 byte of data
 * 
 * There are PC (program counter) and ONZP (overflow, negative, zero, positive) registers
 ****************************************************************/

/****************************************************************
 * Future Work
 * Add push, pop and ret instructions
 * Add SP and BP registers
 * 
 ****************************************************************/

/****************************************************************
 * Arithmetic Ops: ADD, SUB
 * Arithmetic operations are of the form SRC1 = SRC1 <op> SRC2
 * SRC1 is a register r0-r7
 * SRC2 can be a register, or immediate value
 * MODIFIER 0 indicates SRC2 is a register. 1 indicates an immediate value
 * ONZP bits are set based on the result of the operation
 * 
 * | OPCODE | MODE | SRC1 | SRC2 |
 * |  15:12 |  11  | 10:8 |  7:0 |
 *
 * |      OPCODE    | MODE | SRC1 | RESERVED | SRC2 Register |
 * |  0b0000/0b0001 |   0  | 10:8 |    7:3   |      2:0      |
 *
 * |      OPCODE    | MODE | SRC1 | SRC2 Immediate |
 * |  0b0000/0b0001 |   1  | 10:8 |      7:0       |
 ****************************************************************/

#define ADDRESS_SPACE 256

/****************************************************************
 * Masks
 ****************************************************************/
#define OPCODE_MASK 0xF000
#define OPCODE_SHIFT 0xC

// Arithmetic Instruction Masks
#define ARITH_MODIFIER_MASK 0x0800
#define ARITH_MODIFIER_SHIFT 0xB
#define ARITH_SRC1_MASK 0x0700
#define ARITH_SRC1_SHIFT 0x8
#define ARITH_SRC2_IMMEDIATE_MASK 0x00FF
#define ARITH_SRC2_IMMEDIATE_SHIFT 0x0
#define ARITH_SRC2_REGISTER_MASK 0x0007
#define ARITH_SRC2_REGISTER_SHIFT 0x0

/****************************************************************
 * Internal representations of instructions
 ****************************************************************/
typedef enum opcode_e {
    ADD = 0x0,
    SUB,
    INVERT,
    LD,
    STR,
    MOV,
    TEST,
    JUMP,
    HALT,

    /* Add any new opcodes before this line */
    NUM_OPCODES
} opcode_t;

typedef enum arith_mod_e {
    REGISTER = 0x0,
    IMMEDIATE,
    NUM_ARITH_MODS
} arith_mod_t;

typedef struct decoded_inst {
    opcode_t op;
} decoded_inst_t;

typedef struct opcode_map {
    char* name;
    opcode_t opcode;
} opcode_map_t;

typedef enum reg_name_e {
    R0 = 0x0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,

    /* Add any new registers before this line */
    NUM_REGISTERS
} reg_name_t;

typedef uint8_t reg_value_t;
typedef struct reg_file {
    reg_value_t r[NUM_REGISTERS];
    reg_value_t pc;
    reg_value_t onzp : 4;
} reg_file_t;

typedef uint8_t mem_address_t;
typedef uint8_t mem_value_t;
typedef struct mem {
    mem_value_t value[ADDRESS_SPACE];
} mem_t;

typedef struct machine_state {
    reg_file_t register_file;
    mem_t memory;
} machine_state_t;

typedef struct internal_arith_inst {
    uint16_t opcode;
    uint16_t modifier;
    uint16_t src1;
    uint16_t src2_immediate;
    uint16_t src2_register;
} internal_arith_inst_t;

// TODO Figure out flags and modifiers for all the rest of the instructions
typedef struct internal_ldstr_inst {
    uint16_t opcode;
    uint16_t modifier;
    uint16_t reg;
    uint16_t immediate_addr;
} internal_ldstr_inst_t;

typedef struct internal_mov_inst {
    uint16_t opcode;
    uint16_t modifier;
    uint16_t src;
    uint16_t dest;
} internal_mov_inst_t;

typedef struct internal_test_inst {
    uint16_t opcode;
    uint16_t src1;
    uint16_t src2;
} internal_test_inst_t;

typedef struct internal_jump_inst {
    uint16_t opcode;
    uint16_t modifier;
    uint16_t flags;
    uint16_t src;
    uint16_t dest;
} internal_jump_inst_t;

typedef union internal_inst {
    internal_arith_inst_t arith;
    internal_ldstr_inst_t ldstr;
    internal_mov_inst_t mov;
    internal_test_inst_t test;
    internal_jump_inst_t jump;
} internal_inst_t;

/****************************************************************
 * Opcode string name mapping
 ****************************************************************/
extern opcode_map_t opcode_mapping[NUM_OPCODES];
char* get_opcode_name(opcode_t opcode);
void set_opcode_names();

/****************************************************************
 * Actual instruction structs
 ****************************************************************/
typedef uint16_t inst_t;

/****************************************************************
 * Functions to extract instruction information
 ****************************************************************/
bool extract_instruction(inst_t, internal_inst_t*);
bool extract_arith(inst_t, internal_inst_t*);

opcode_t extract_opcode(inst_t);

/****************************************************************
 * Functions to execute instruction functionality
 ****************************************************************/
void do_add(internal_inst_t*);
void do_add_reg(internal_inst_t*);
void do_add_imm(internal_inst_t*);

void do_sub(internal_inst_t*);
void do_sub_reg(internal_inst_t*);
void do_sub_imm(internal_inst_t*);

// TODO Add rest of instructions

/****************************************************************
 * Internal emulated machine state
 ****************************************************************/
extern machine_state_t state;

/****************************************************************
 * Manipulate internal emulated machine state
 ****************************************************************/
reg_value_t increment_pc();
reg_value_t decrement_pc();
reg_value_t read_pc();
void write_pc(reg_value_t);

reg_value_t read_reg(reg_name_t);
void write_reg(reg_name_t, reg_value_t);

reg_value_t read_onzp();
void write_onzp(reg_value_t);

mem_value_t read_mem(mem_address_t);
void write_mem(mem_address_t, mem_value_t);

/****************************************************************
 * Program operations
 ****************************************************************/
bool load_program(void*, uint16_t);
bool run_program(uint16_t);
bool execute_instruction(inst_t);

#endif
