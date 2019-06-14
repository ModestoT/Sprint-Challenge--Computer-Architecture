#include "cpu.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define DATA_LEN 1024
#define SP 7  // Stack pointer index value
/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *filename)
{
  FILE *fp; // File pointer
  char data[DATA_LEN];
  int address = 0; // index of current position in memory array

  // TODO: Replace this with something less hard-coded
  fp = fopen(filename, "r");

  if (fp == NULL){
    fprintf(stderr, "File not found please try again");
    exit(1);
  }

  // loop through the file to add the data to memory
  while(fgets(data, DATA_LEN, fp) != NULL){
    char *endptr;
    // convert the string data in the file into a number value in base 2
    unsigned char val = strtoul(data, &endptr, 2);

    // if the endptr pointer value is equal to the memory address of the start of the data skip that line
    if (endptr == data){
      continue;
    }

    // add the converted data value to the memory array
    cpu->ram[address] = val;
    address++;
  }
}

unsigned char cpu_ram_read(struct cpu *cpu, int index)
{
  return cpu->ram[index];
};

void cpu_ram_write(struct cpu *cpu, int index, int value)
{
  cpu->ram[index] = value;
};

/**
 * ALU
 */
void alu(struct cpu *cpu, enum alu_op op, unsigned char regA, unsigned char regB)
{
  switch (op) {
    case ALU_MUL:
      // TODO
      cpu->registers[regA] = cpu->registers[regA] * cpu->registers[regB];
      break;

    // TODO: implement more ALU ops
    case ALU_ADD:
      cpu->registers[regA] = cpu->registers[regA] + cpu->registers[regB];
      break;

    case ALU_CMP:
      // 00000LGE
      if (cpu->registers[regA] == cpu->registers[regB]){
        // printf("Equal to\n");
        cpu->flag = (cpu->flag | 0b00000001);
      } else if (cpu->registers[regA] < cpu->registers[regB]){
        // printf("Less than\n");
        cpu->flag = (cpu->flag | 0b00000100);
      } else {
        // printf("Greater than\n");
        cpu->flag = (cpu->flag | 0b00000010);
      }
      break;
    
    case ALU_INC:
      cpu->registers[regA]++;
      break;
    case ALU_DEC:
      cpu->registers[regA]--;
      break;

    default:
      printf("Unknown instruction %02x at address %02x\n", op, cpu->pc);
      exit(1);
  }
}

void stack_push(struct cpu *cpu, unsigned char val){
  // decrement the Stack Pointer 
  cpu->registers[SP]--;
  // Copy the value in the given register to the address pointed to by SP
  cpu->ram[cpu->registers[SP]] = val;
}

unsigned char stack_pop(struct cpu *cpu, unsigned char reg){
  // Copy the value from the address pointed to by SP to the given register.
  unsigned char popped = cpu->ram[cpu->registers[SP]];
  cpu->registers[reg] = popped;
  // Increment SP.
  cpu->registers[SP]++;
  
  return popped;
}

// void trace(struct cpu *cpu)
// {
//     printf("%02X | ", cpu->pc);

//     printf("%02X %02X %02X |",
//         cpu_ram_read(cpu, cpu->pc),
//         cpu_ram_read(cpu, cpu->pc + 1),
//         cpu_ram_read(cpu, cpu->pc + 2));

//     for (int i = 0; i < 8; i++) {
//         printf(" %02X", cpu->registers[i]);
//     }

//     printf("\n");
// }

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction

  while (running) {
    // TODO
    // printf("flag:%02X\n", (cpu->flag | 0b00000100 ));
    // 1. Get the value of the current instruction (in address PC).
    unsigned char ir = cpu_ram_read(cpu, cpu->pc);
    // 2. Figure out how many operands this next instruction requires
    int num_operands = (ir >> 6) + 1;
    // creates a mask that compares the 4th bit to check if this instruction sets the PC or not, number that is not 0 means it does
    int is_setting_pc = ((ir) & (1 << 4));
    // 3. Get the appropriate value(s) of the operands following this instruction
    unsigned char operandA = cpu_ram_read(cpu, cpu->pc+1); 
    unsigned char operandB = cpu_ram_read(cpu, cpu->pc+2);
    int val;
    int index;
    // trace(cpu);
    // 4. switch() over it to decide on a course of action.
    switch (ir) {
      // 5. Do whatever the instruction should do according to the spec.

      case CMP:
        alu(cpu, ALU_CMP, operandA, operandB);
        break;

      case JEQ:
        // printf("%02X\n", (cpu->flag) & 0b00000001);
        if ( (cpu->flag) & 0b00000001){
          cpu->pc = cpu->registers[operandA];
        } else {
          cpu->pc += num_operands;  
        }
        break;
      
      case LD:
        cpu->registers[operandA] = cpu->ram[cpu->registers[operandB]];
        break;

      case PRA:
        printf("%c",cpu->registers[operandA]);
        break;
      
      case INC:
        alu(cpu, ALU_INC, operandA, operandB);
        break;
      
      case DEC:
        alu(cpu, ALU_DEC, operandA, operandB);
        break;
      
      case JMP:
        cpu->pc = cpu->registers[operandA];
        break;
      // LDI instruction, Saves a value to the provided register index
      case LDI:
        index = operandA;
        val = operandB;
        cpu->registers[index] = val;
        break;

      // PRN instruction, prints the value held in the provided register index
      case PRN:
        index = operandA;
        printf("%d\n", cpu->registers[index]);
        break;

      // HLT instruction, stops the program
      case HLT:
        running = 0;
        break;

      // MUL instruction, multiplies two values from the provided register indices
      case MUL:
        alu(cpu, ALU_MUL, operandA, operandB);
        break;
      // PUSH instruction, pushes the value from the provided register index onto the Stack 
      case PUSH:
        stack_push(cpu, cpu->registers[operandA]);
        break;

      // POP instruction, pops the value from the stack onto the provided register index 
      case POP:
        stack_pop(cpu, operandA);
        break;

      // CALL instruction, have to program run the instructions at the provided memory address, pushes the return address onto the stack
      case CALL:
        // push the next instruction address after the CALL instruction onto the stack
        index = cpu->pc + 2;
        stack_push(cpu, index);
        // set the PC to the subroutine instruction address
        cpu->pc = cpu->registers[operandA];
        break;
      
      // RET instruction, has the program go back to the return address saved on the stack
      case RET:
        // pop the address saved from CALL instruction from the stack
        index = stack_pop(cpu, operandA);
        // set the PC to the address that was poped from the stack
        cpu->pc = index;
        break;

      // ADD instruction, adds two values stored in the provided register indices together
      case ADD:
        alu(cpu, ALU_ADD, operandA, operandB);
        break;

      default:
        printf("Unknown instruction %02x at address %02x\n", ir, cpu->pc);
        exit(1);
    }
    // 6. Move the PC to the next instruction.
    if (is_setting_pc == 0){
      cpu->pc+=num_operands;
    }
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  // TODO: Initialize the PC and other special registers

  // set cpu PC to 0
  cpu->pc = 0;
  // set cpu flag to 0
  cpu->flag = 0b00000000;
  // Set cpu registers array to be all 0's 
  memset(cpu->registers, 0, sizeof(cpu->registers));
  // set cpu register 7 as the stack pointer
  cpu->registers[SP] = 0xF4;
  // set cpu Memory array to be all 0's
  memset(cpu->ram, 0, sizeof(cpu->ram));
}
