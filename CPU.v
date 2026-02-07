module Shifter(
    input  [31:0] A,           // Kaynak veri (örn. rs1 içeriği)
    input  [4:0]  ShiftAmount, // Kaydırma miktarı (rs2[4:0])
    output [31:0] Result       // SLL sonucu
);
    assign Result = A << ShiftAmount;
endmodule

module ALU(
    input  [31:0] A,
    input  [31:0] B,
    input  [2:0]  ALUControl,
    output reg [31:0] Result,
    output Zero
);
    always @(*) begin
        case (ALUControl)
            3'b000: Result = A + B;           // ADD
            3'b001: Result = A - B;           // SUB
            3'b010: Result = A & B;           // AND
            3'b011: Result = A | B;           // OR
            3'b101: Result = (A < B) ? 32'b1 : 32'b0;   // SLT
            default: Result = 32'b0;
        endcase
    end
    assign Zero = (Result == 32'b0);
endmodule

module RegFile(
    input         clk,
    input         RegWrite,
    input  [4:0]  rs1,
    input  [4:0]  rs2,
    input  [4:0]  rd,
    input  [31:0] WriteData,
    output [31:0] ReadData1,
    output [31:0] ReadData2
);
    reg [31:0] reg_array [0:31];
    integer i;
    
    initial begin
        for(i = 0; i < 32; i = i + 1)
            reg_array[i] = 32'b0;
    end

    assign ReadData1 = (rs1 == 5'b00000) ? 32'b0 : reg_array[rs1];
    assign ReadData2 = (rs2 == 5'b00000) ? 32'b0 : reg_array[rs2];

    always @(posedge clk) begin
        if (RegWrite && (rd != 5'b00000))
            reg_array[rd] <= WriteData;
    end
endmodule

module InstructionMemory(
    input  [31:0] addr,
    output [31:0] instr
);
    reg [31:0] mem [0:127];
    integer i;
    
    initial begin
        
        mem[0] = 32'b00000000101000000000000010010011;   // addi x1, x0, 10  ; x1 = 10
        mem[1] = 32'b00000001010000000000000100010011;   // addi x2, x0, 20  ; x2 = 20
        mem[2] = 32'b00000000001000001000000110110011;   // add  x3, x1, x2  ; x3 = x1 + x2 = 30
        mem[3] = 32'b01000000000100011000001000110011;   // sub  x4, x3, x1  ; x4 = x3 - x1 = 20
        mem[4] = 32'b00000000001000001010001010110011;   // slt  x5, x1, x2  ; x5 = (10 < 20) ? 1 : 0
        mem[5] = 32'b00000000001000001110001100110011;   // or   x6, x1, x2  ; x6 = x1 OR x2
        mem[6] = 32'b00000000001000001111001110110011;   // and  x7, x1, x2  ; x7 = x1 AND x2
        mem[7] = 32'b00000000001100000010000000100011;   // sw   x3, 0(x0)   ; mem[0] = x3 (30)
        mem[8] = 32'b00000000000000000010010000000011;   // lw   x8, 0(x0)   ; x8 = mem[0] (30)
        mem[9] = 32'b00000001100100001010000100010011;   // slti x10, x2, 25 ; x10 = (20 < 25) ? 1 : 0
        mem[10] = 32'b00000000111100001110010110010011;  // ori  x11, x1, 15 ; x11 = x1 OR 15
        mem[11] = 32'b00000000111100001111101100010011;  // andi x12, x1, 15 ; x12 = x1 AND 15
        mem[12] = 32'b00000000001000001001001010110011;   // sll x5, x1, x2
        mem[13] = 32'b00010010001101000101001010110111;   // lui x5, 0x12345
        mem[14] = 32'b00000000000100001000010001100011; // beq  x1, x1, +8  ; branch taken
        mem[16] = 32'b00000001000000000000010011101111;   //jal      
        
        // Geri kalan belleği sıfırla
        for (i = 17; i < 128; i = i + 1)
            mem[i] = 32'b0;
    end
    
    assign instr = mem[addr[8:2]];
endmodule


module DataMemory(
    input         clk,
    input         MemWrite,
    input  [31:0] addr,
    input  [31:0] writeData,
    output reg [31:0] readData
);
    reg [31:0] dmem [0:127];
    integer i;
    initial begin
        for(i = 0; i < 128; i = i + 1)
            dmem[i] = 32'b0;
    end
    always @(posedge clk) begin
        if (MemWrite)
            dmem[addr[8:2]] <= writeData;
    end
    always @(*) begin
        readData = dmem[addr[8:2]];
    end
endmodule

// Artık Imm_src 3-bit; 3'b100 LUI için U-type formatını sağlar.
module Extend(
    input  [24:0] instr_part,  
    input  [2:0]  Imm_src,   
    output reg [31:0] Immext  
);
    always @(*) begin
        case (Imm_src)
            3'b000: Immext = {{20{instr_part[24]}}, instr_part[24:13]};  
                    // I‑type: orijinal instr[31:20]
            3'b001: Immext = {{20{instr_part[24]}}, instr_part[24:18], instr_part[4:0]};
                    // S‑type: {instr[31:25], instr[11:7]}
            3'b010: Immext = {{19{instr_part[24]}}, instr_part[24], instr_part[0], 
                               instr_part[23:18], instr_part[4:1], 1'b0};
                    // B‑type  
            3'b011: Immext = {{11{instr_part[24]}}, instr_part[24], instr_part[12:5], 
                               instr_part[13], instr_part[23:14], 1'b0};
                    // J‑type  
            3'b100: Immext = {instr_part[24:5], 12'b0};
                    // U‑type (LUI): Üst 20 bit, alt 12 bit sıfır
            default: Immext = 32'b0;
        endcase
    end
endmodule

module ControlUnit(
    input  [6:0] op,
    input  [2:0] funct3,
    input        funct7,
    input        zero,
    output reg   PCsrc,       // 1-bit: 0: PC+4, 1: PC+Immext (branch/jump)
    output reg [2:0] resultSrc,   // 3-bit: 000: ALU, 001: Memory, 010: PC+4, 011: Shifter, 100: LUI
    output reg       MemWrite,
    output reg [2:0] ALUControl,  // 3-bit ALU kontrol sinyali
    output reg       ALUSrc,
    output reg [2:0] Immsrc,      // 3-bit: 000: I-type, 001: S-type, 010: B-type, 011: J-type, 100: LUI
    output reg       RegWrite
);
    always @(*) begin
        // Varsayılan değerler
        RegWrite   = 1'b0;
        ALUSrc     = 1'b0;
        MemWrite   = 1'b0;
        Immsrc     = 3'b000;
        resultSrc  = 3'b000;
        PCsrc      = 1'b0;  // 0: standart PC+4, 1: branch/jump (PC+Immext)
        ALUControl = 3'b000;
        
        case(op)
            // R-Type: add, sub, sll, and, or, slt (opcode = 7'b0110011)
            7'b0110011: begin
                RegWrite   = 1'b1;
                ALUSrc     = 1'b0;
                MemWrite   = 1'b0;
                Immsrc     = 3'b000;       // Immediate kullanılmaz
                PCsrc      = 1'b0;         // Normal PC+4
                case(funct3)
                    3'b000: begin 
                              ALUControl = (funct7) ? 3'b001 : 3'b000; // SUB ya da ADD
                              resultSrc = 3'b000; // ALU sonucu
                            end
                    3'b001: begin 
                              // SLL komutu: şifter modülü devreye girer
                              ALUControl = 3'b000; // ALU kontrol değeri kullanılmayabilir
                              resultSrc = 3'b011;  // 011: Shifter sonucu
                            end
                    3'b010: begin 
                              ALUControl = 3'b101; // SLT
                              resultSrc = 3'b000;
                            end
                    3'b110: begin 
                              ALUControl = 3'b011; // OR
                              resultSrc = 3'b000;
                            end
                    3'b111: begin 
                              ALUControl = 3'b010; // AND
                              resultSrc = 3'b000;
                            end
                    default: begin
                              ALUControl = 3'b000;
                              resultSrc = 3'b000;
                             end
                endcase
            end

            // I-Type aritmetik (addi, andi, ori, slti): opcode = 7'b0010011
            7'b0010011: begin
                RegWrite   = 1'b1;
                ALUSrc     = 1'b1;
                MemWrite   = 1'b0;
                Immsrc     = 3'b000;       // I‑type immediate
                resultSrc  = 3'b000;       // ALU sonucu
                PCsrc      = 1'b0;
                case(funct3)
                    3'b000: ALUControl = 3'b000; // ADDI
                    3'b111: ALUControl = 3'b010; // ANDI
                    3'b110: ALUControl = 3'b011; // ORI
                    3'b010: ALUControl = 3'b101; // SLTI
                    default: ALUControl = 3'b000;
                endcase
            end

            // Load Word (lw): opcode = 7'b0000011
            7'b0000011: begin
                RegWrite   = 1'b1;         // Hafızadan okunan veri yazılır
                ALUSrc     = 1'b1;         
                MemWrite   = 1'b0;
                Immsrc     = 3'b000;        // lw I‑type formatında
                resultSrc  = 3'b001;        // 001: Hafızadan okunan veri
                PCsrc      = 1'b0;
                ALUControl = 3'b000;        // ADD (adres hesaplaması)
            end

            // Store Word (sw): opcode = 7'b0100011
            7'b0100011: begin
                RegWrite   = 1'b0;
                ALUSrc     = 1'b1;
                MemWrite   = 1'b1;         // Yazma aktif
                Immsrc     = 3'b001;        // S‑type format
                resultSrc  = 3'b000;        // MUX kullanımı gerekmez
                PCsrc      = 1'b0;
                ALUControl = 3'b000;        // ADD
            end

            // Branch (beq): opcode = 7'b1100011
            7'b1100011: begin
                RegWrite   = 1'b0;
                ALUSrc     = 1'b0;
                MemWrite   = 1'b0;
                Immsrc     = 3'b010;        // B‑type format
                resultSrc  = 3'b000;
                PCsrc      = (zero) ? 1'b1 : 1'b0;  // Eğer koşul sağlanırsa PC+Immext
                ALUControl = 3'b001;        // SUB
            end

            // Jump (jal): opcode = 7'b1101111
            7'b1101111: begin
                RegWrite   = 1'b1;         // Link (PC+4) yazılır
                ALUSrc     = 1'b0;
                MemWrite   = 1'b0;
                Immsrc     = 3'b011;        // J‑type format
                resultSrc  = 3'b010;        // 010: PC+4
                PCsrc      = 1'b1;          // Jump: PC = PC + Immext
                ALUControl = 3'b000;
            end
            
            // LUI: opcode = 7'b0110111
            7'b0110111: begin
                RegWrite   = 1'b1;
                ALUSrc     = 1'b1;        
                MemWrite   = 1'b0;
                Immsrc     = 3'b100;        // U‑type (LUI)
                resultSrc  = 3'b100;        // 100: LUI sonucu
                PCsrc      = 1'b0;          // Normal PC+4
                ALUControl = 3'b000;        // Kullanılmayacak
            end

            default: begin
                RegWrite   = 1'b0;
                ALUSrc     = 1'b0;
                MemWrite   = 1'b0;
                Immsrc     = 3'b000;
                resultSrc  = 3'b000;
                PCsrc      = 1'b0;
                ALUControl = 3'b000;
            end
        endcase
    end
endmodule


// 3-bit resultSrc: 000 → ALU, 001 → MemData, 010 → PC+4, 011 → Shifter, 100 → LUI
module ResultMux(
    input  [2:0] resultSrc,
    input  [31:0] ALUResult,
    input  [31:0] MemData,
    input  [31:0] PcPlus4,
    input  [31:0] ShifterResult,
    input  [31:0] LuiResult,
    output reg [31:0] Result
);
    always @(*) begin
        case(resultSrc)
            3'b000: Result = ALUResult;
            3'b001: Result = MemData;
            3'b010: Result = PcPlus4;
            3'b011: Result = ShifterResult;
            3'b100: Result = LuiResult;
            default: Result = ALUResult;
        endcase
    end
endmodule

module SingleCycleCPU(
    input clk
);
    // Program Sayacı
    reg [31:0] PC;
    
    // Instruction Memory'dan gelen talimat
    wire [31:0] instr;
    
    // Talimattan ayrıştırılan alanlar
    wire [6:0] op;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    wire       funct7;
    
    assign op     = instr[6:0];
    assign rd     = instr[11:7];
    assign funct3 = instr[14:12];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign funct7 = instr[30];
    
    wire [24:0] instr_part;
    assign instr_part = instr[31:7];
    
    // Kontrol Ünitesi ve diğer sinyaller
    // PCsrc artık 1-bit: 0 → PC+4, 1 → PC+ImmExt (PCtarget)
    wire        PCsrc;
    wire [2:0]  resultSrc;
    wire        MemWrite;
    wire [2:0]  ALUControl;
    wire        ALUSrc;
    wire [2:0]  Immsrc;
    wire        RegWrite;
    
    // Extend modülünden gelen ImmExt (değişiklik yapılmadı)
    wire [31:0] ImmExt;
    
    // Register File çıkışları
    wire [31:0] ReadData1, ReadData2;
    
    // ALU sonucu ve Zero sinyali
    wire [31:0] ALUResult;
    wire        Zero;
    
    // Data Memory'den okunan veri
    wire [31:0] MemData;
    
    // PC hesaplamaları
    wire [31:0] PcPlus4;
    wire [31:0] PCtarget;  // PCtarget = PC + ImmExt
    wire [31:0] nextPC;
    
    // Shifter modülü sonucu
    wire [31:0] ShifterResult;
    
    // ResultMux çıkışı (Register File'ın WriteData girişine gidecek)
    wire [31:0] WriteData;
    
    // ---------------- Modul Örnekleştirmeleri ----------------
    
    InstructionMemory IMem (
        .addr(PC),
        .instr(instr)
    );
    
    ControlUnit CU (
        .op(op),
        .funct3(funct3),
        .funct7(funct7),
        .zero(Zero),
        .PCsrc(PCsrc),
        .resultSrc(resultSrc),
        .MemWrite(MemWrite),
        .ALUControl(ALUControl),
        .ALUSrc(ALUSrc),
        .Immsrc(Immsrc),
        .RegWrite(RegWrite)
    );
    
    Extend ext (
        .instr_part(instr_part),
        .Imm_src(Immsrc),
        .Immext(ImmExt)
    );
    
    RegFile rf (
        .clk(clk),
        .RegWrite(RegWrite),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .WriteData(WriteData),
        .ReadData1(ReadData1),
        .ReadData2(ReadData2)
    );
    
    // İkinci operand: ALUSrc kontrolü ile seçimi
    wire [31:0] ALUB;
    assign ALUB = ALUSrc ? ImmExt : ReadData2;
    
    ALU alu_inst (
        .A(ReadData1),
        .B(ALUB),
        .ALUControl(ALUControl),
        .Result(ALUResult),
        .Zero(Zero)
    );
    
    Shifter shifter_inst (
        .A(ReadData1),
        .ShiftAmount(ReadData2[4:0]),
        .Result(ShifterResult)
    );
    
    DataMemory DM (
        .clk(clk),
        .MemWrite(MemWrite),
        .addr(ALUResult),
        .writeData(ReadData2),
        .readData(MemData)
    );
    
    assign PcPlus4 = PC + 32'd4;
    assign PCtarget = PC + ImmExt;
    assign nextPC = (PCsrc == 1'b0) ? PcPlus4 : PCtarget;
    
    ResultMux rm_inst (
        .resultSrc(resultSrc),
        .ALUResult(ALUResult),
        .MemData(MemData),
        .PcPlus4(PcPlus4),
        .ShifterResult(ShifterResult),
        .LuiResult(ImmExt),  // LUI durumu için Extend'den gelen ImmExt kullanılıyor
        .Result(WriteData)
    );
    
    always @(posedge clk)
        PC <= nextPC;
    
    initial begin
        PC = 32'b0;
    end

endmodule
