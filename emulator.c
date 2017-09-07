/****************************************************************
 * Author: Eric Mackay
 * Date: August 9, 2017
 * Project: TinyEmulator - An Emulator for TinyCPU
 * File: emulator.c
 ****************************************************************/

#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "emulator.h"

/****************************************************************
 * IOpcade to string name mapping
 ****************************************************************/
opcode_map_t opcode_mapping[NUM_OPCODES];

/****************************************************************
 * Internal emulated machine state
 ****************************************************************/
machine_state_t state;

/****************************************************************
 * Returns the string version of the given opcode
 * Note: set_opcode_names() must be called first
 ****************************************************************/
char* get_opcode_name(opcode_t opcode) {
    char* name = NULL;

    if (opcode < NUM_OPCODES) {
        name = opcode_mapping[opcode].name;
    }

    return name;
}

/****************************************************************
 * Sets up the opcode name mapping
 ****************************************************************/
void set_opcode_names() {
    opcode_t opcode;
    char* name;

    struct init_opcode_names {
        char* name;
        opcode_t opcode;
    }  init[] = {
        {"ADD"      , ADD},
        {"SUB"      , SUB},
        {"INVERT"   , INVERT},
        {"LD"       , LD},
        {"STR"      , STR},
        {"MOV"      , MOV},
        {"TEST"     , TEST},
        {"JUMP"     , JUMP},
        {"HALT"     , HALT},
    };

#define INIT_SIZE(A) sizeof(A)/sizeof(struct init_opcode_names)

    for (unsigned int index = 0; index < INIT_SIZE(init); index++) {
        opcode = init[index].opcode;
        name = init[index].name;
        opcode_mapping[opcode].name = name;
    }
#undef INIT_SIZE
}

/****************************************************************
 * Functions to extract instruction information
 ****************************************************************/
bool extract_instruction(inst_t input, internal_inst_t* output) {
    opcode_t opcode;
    bool success = false;

    memset(output, 0, sizeof(internal_inst_t));
    opcode = extract_opcode(input);
    output->arith.opcode = opcode;

    switch (opcode) {
        case ADD:
        case SUB:
            success = extract_arith(input, output);
            break;
        case INVERT:
        case LD:
        case STR:
        case MOV:
        case TEST:
        case JUMP:
        case HALT:
            printf("Opcode not supported yet\n");
            break;
        default:
            printf("Could not extract opcode from instruction\n");
            break;
    }

    return success;
}

bool extract_arith(inst_t input, internal_inst_t* output) {
    opcode_t opcode = extract_opcode(input);

    if (opcode != ADD && opcode != SUB) {
        printf("Invalid opcode for extract_arith()");
        return false;
    }

    output->arith.modifier = input &
                             ARITH_MODIFIER_MASK >> 
                             ARITH_MODIFIER_SHIFT;

    output->arith.src1 = input & 
                         ARITH_SRC1_MASK >> 
                         ARITH_SRC1_SHIFT;
    
    if (output->arith.modifier) {
        output->arith.src2_immediate = input & 
                                       ARITH_SRC2_IMMEDIATE_MASK >> 
                                       ARITH_SRC2_IMMEDIATE_SHIFT;
    } else {
        output->arith.src2_register = input & 
                                      ARITH_SRC2_REGISTER_MASK >> 
                                      ARITH_SRC2_REGISTER_SHIFT;
    }

    return output;
}

opcode_t extract_opcode(inst_t input) {
    uint16_t extract = input & 
                       OPCODE_MASK >> 
                       OPCODE_SHIFT;

    return (opcode_t)extract;
}

/****************************************************************
 * Functions to execute instruction functionality
 ****************************************************************/
void do_add(internal_inst_t* instruction) {

    // Immediate add
    if (instruction->arith.modifier) {
        do_add_imm(instruction);

    // Register + Register
    } else {
        do_add_reg(instruction);
    }
}

void do_add_reg(internal_inst_t* instruction) {
    reg_value_t result;
    result = read_reg(instruction->arith.src1)
             + read_reg(instruction->arith.src2_register);
    write_reg(instruction->arith.src1, result);
}

void do_add_imm(internal_inst_t* instruction) {
    reg_value_t result;
    result = read_reg(instruction->arith.src1)
             + instruction->arith.src2_immediate;
    write_reg(instruction->arith.src1, result);
}

/*void do_sub(internal_inst_t* instruction);
void do_sub_reg(internal_inst_t* instruction);
void do_sub_imm(internal_inst_t* instruction);*/

/****************************************************************
 * Manipulate internal emulated machine state
 ****************************************************************/
/*reg_value_t increment_pc();
reg_value_t decrement_pc();
reg_value_t read_pc();
void write_pc(reg_value_t);*/

reg_value_t read_reg(reg_name_t reg) {
    if (reg < NUM_REGISTERS) {
        printf("Read R%u\n", reg);
        return state.register_file.r[reg];
    }

    return 0;
}

void write_reg(reg_name_t reg, reg_value_t value) {
    if (reg < NUM_REGISTERS) {
        printf("Write R%u, Value: %u\n", reg, value);
        printf("Before write: %u\n", state.register_file.r[reg]);
        state.register_file.r[reg] = value;
        printf("After write: %u\n", state.register_file.r[reg]);
    }
}

/*reg_value_t read_onzp();
void write_onzp(reg_value_t);

mem_value_t read_mem(mem_address_t);
void write_mem(mem_address_t, mem_value_t);*/


/****************************************************************
 * Program operations
 ****************************************************************/
bool load_program(void* start, uint16_t length) {
    if (length > ADDRESS_SPACE) {
        return false;
    }

    memcpy(state.memory.value, start, length);
    return true;
}

bool run_program(uint16_t start) {
    return true;
}

bool execute_instruction(inst_t input) {
    internal_inst_t instruction;
    if (!extract_instruction(input, &instruction)) {
        printf("Error extracting instruction 0x%x\n", input);
        return false;
    }

    //TODO More
    return true;
}


/****************************************************************
 * Main
 ****************************************************************/
int main(int argc, char** argv) {
    //inst_t start;
    inst_t program[6];

    // Initialize opcode name mapping
    set_opcode_names();

    printf("Welcome to the TinyEmulator!\n");

    memset(program, 0, 6);

    //TODO bitfields not laid out as expected
    // Casting will probably not work right! Ugh
    //Pulling out bitfields from raw bytes will definitely not work
    // Need to just have generic inst type and then pull things out
    // to internal representation using masks.

    // Instruction 1: R0 = 0 + 3
    /*add_inst = (arith_inst_t*)&program[0];
    add_inst->opcode = ADD;
    add_inst->modifier = IMMEDIATE;
    add_inst->src1 = R0;
    add_inst->src2.immediate = 0x3;

    printf("Instruction 0: 0x%x\n", *(uint16_t*)program);

    // Instruction 2: R1 = 0 + 5;
    add_inst = (arith_inst_t*)&program[1];
    add_inst->opcode = ADD;
    add_inst->modifier = IMMEDIATE;
    add_inst->src1 = R1;
    add_inst->src2.immediate = 0x5;

    printf("Instruction 1: 0x%x\n", *((uint16_t*)program + 1));

    // Instruction 3: R0 = 3 + 5
    add_inst = (arith_inst_t*)&program[2];
    add_inst->opcode = ADD;
    add_inst->modifier = REGISTER;
    add_inst->src1 = R0;
    add_inst->src2.reg.src2_reg = R1;

    printf("Instruction 2: 0x%x\n", *((uint16_t*)program + 2));

    if (!load_program((void*)program, 6)) {
        printf("Could not load program!\n");
        return -1;
    }

    start = 0;

    if (!run_program(start)) {
        printf("Could not run program!\n");
        return -1;
    }*/

    /*do_add((arith_inst_t*)&program[0]);
    do_add((arith_inst_t*)&program[1]);
    do_add((arith_inst_t*)&program[2]);*/

    printf("R0: %u\n", read_reg(R0));
    printf("R1: %u\n", read_reg(R1));

    /*if (argc != 2) {
        printf("Invalid usage! Usage: ./TinyEmulator <path to input file>\n");
        return -1;
    }*/

    //printf("Input: %s\n", argv[1]);

    return 0;
}