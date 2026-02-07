# Single-Cycle RISC-V Processor (Verilog)

From-scratch implementation of a 32-bit single-cycle RISC-V processor in Verilog.

This project includes the complete datapath, control unit, register file, ALU, instruction/data memories and additional hardware blocks required to execute a subset of the RV32I instruction set.

The design was built for educational and architectural understanding purposes and verified using simulation and waveform analysis.

---

## 🚀 Features

Supported instructions:

### Arithmetic / Logic
- add
- sub
- and
- or
- slt
- addi
- andi
- ori
- slti

### Memory
- lw
- sw

### Control Flow
- beq
- jal

### Extended Instructions
- sll (Shift Left Logical) → dedicated Shifter module
- lui (Load Upper Immediate) → U-type immediate support

---

## 🧠 Architecture

Single-cycle architecture:
- Each instruction completes in one clock cycle
- Separate instruction and data memories
- Combinational control logic

### Major Components

- **ALU** → arithmetic & logical operations
- **Register File** → 32 × 32-bit registers (x0 hardwired to 0)
- **Control Unit** → opcode-based control signal generation
- **Immediate Extend Unit** → I/S/B/J/U type formats
- **Instruction Memory**
- **Data Memory**
- **Shifter (SLL support)**
- **Result Mux** → selects write-back source
- **Program Counter logic**

---

## 📂 Module Structure

