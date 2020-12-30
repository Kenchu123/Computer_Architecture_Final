// Your code

module RISCV(clk,
            rst_n,
            // for mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // for mem_I
            mem_addr_I,
            mem_rdata_I
            );

    // Input/Output Part
    input             clk, rst_n;   // Clock signal, Active low asyncrhonous reset signal
    input      [31:0] mem_rdata_I;  // Instruction read from instruction memory
    output reg [31:2] mem_addr_I;   // Output address of instruction memory
    output     [31:2] mem_addr_D;   // Output address of data memory
    output            mem_wen_D;    // Write-enable
    output     [63:0] mem_wdata_D;  // Data store to data memory
    input      [63:0] mem_rdata_D;  // Data read from data memory

    // Parameter Part
    // 23-bit instruction_type
    // { JAL, JALR, BEQ, BNE, LD, SD, ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI, ADD, SUB, SLL, SLT, XOR, SRL, SRA, OR, AND }
    // 12-bit ctrl_signal
    // { JAL, JALR, Branch, MemRead, MemWrite, MemtoReg, RegWrite, ALUSrc, ALUOp }
    parameter R = 5'b10000;
    parameter I = 5'b01000;
    parameter S = 5'b00100;
    parameter B = 5'b00010;
    parameter J = 5'b00001;

    parameter JAL  = {1'b1, 22'b0};             // 1101111
    parameter JALR = {1'b0, 1'b1, 21'b0};       // 1100111 000
    parameter BEQ  = {2'b0, 1'b1, 20'b0};       // 1100011 000
    parameter BNE  = {3'b0, 1'b1, 19'b0};       // 1100011 001
    parameter LD   = {4'b0, 1'b1, 18'b0};       // 0000011 011
    parameter SD   = {5'b0, 1'b1, 17'b0};       // 0100011 011
    parameter ADDI = {6'b0, 1'b1, 16'b0};       // 0010011 000
    parameter SLTI = {7'b0, 1'b1, 15'b0};       // 0010011 010
    parameter XORI = {8'b0, 1'b1, 14'b0};       // 0010011 100
    parameter ORI  = {9'b0, 1'b1, 13'b0};       // 0010011 110 
    parameter ANDI = {10'b0, 1'b1, 12'b0};      // 0010011 111
    parameter SLLI = {11'b0, 1'b1, 11'b0};      // 0010011 001 0000000
    parameter SRLI = {12'b0, 1'b1, 10'b0};      // 0010011 101 0000000
    parameter SRAI  = {13'b0, 1'b1, 9'b0};      // 0010011 101 0100000
    parameter ADD  = {14'b0, 1'b1, 8'b0};       // 0110011 000 0000000
    parameter SUB  = {15'b0, 1'b1, 7'b0};       // 0110011 000 0100000
    parameter SLL  = {16'b0, 1'b1, 6'b0};       // 0110011 001 0000000
    parameter SLT  = {17'b0, 1'b1, 5'b0};       // 0110011 010 0000000
    parameter XOR  = {18'b0, 1'b1, 4'b0};       // 0110011 100 0000000
    parameter SRL  = {19'b0, 1'b1, 3'b0};       // 0110011 101 0000000
    parameter SRA  = {20'b0, 1'b1, 2'b0};       // 0110011 101 0100000
    parameter OR   = {21'b0, 1'b1, 1'b0};       // 0110011 110 0000000
    parameter AND  = {22'b0, 1'b1};             // 0110011 111 0000000

    // Wire Part
    wire [22:0] type;    // JAL, JALR ... 23 bits
	wire [4:0]  format;  // R, I, S, B, J. 5 bits
    wire [11:0] ctrl;    // control signal
    wire [63:0] imm;     // immediate value
        
    // ctrl signal
    wire        Jal;
    wire        Jalr;
    wire        Branch;
    wire        MemRead;
    wire        MemWrite;
    wire        MentoReg;
    wire        RegWrite;
    wire        ALUSrc;
    wire [3:0]  ALUOp;

    wire [63:0] read_data1, read_data2;
    wire [4:0]  read_reg1, read_reg2, write_reg;
    wire [63:0] write_data;
    wire [63:0] mem_rdata_D_BE; // Big Endian
    wire [63:0] mem_wdata_D_LE; // Little Endian

    wire [63:0] reg_or_imm; // ALU data2
    wire [63:0] ALU_result;
    wire        zero;
    wire [3:0]  ALU_ctrl;

    wire [63:0] shifted_imm;

    // Reg Part
    reg  [29:0] next_mem_addr_I;
    
    // Assign Part

    // assign ctrl signal
    assign Jal      =   ctrl[11];
    assign Jalr     =   ctrl[10];
    assign Branch   =   ctrl[9];
    assign MemRead  =   ctrl[8];
    assign MemWrite =   ctrl[7];
    assign MemtoReg =   ctrl[6];
    assign RegWrite =   ctrl[5];
    assign ALUSrc   =   ctrl[4];
    assign ALUOp    =   ctrl[3:0];

    // assign others
    assign read_reg1 = {mem_rdata_I[11:8], mem_rdata_I[23]};   // Little Endian
    assign read_reg2 = {mem_rdata_I[0], mem_rdata_I[15:12]};   // Little Endian
    assign write_reg = {mem_rdata_I[19:16], mem_rdata_I[31]};  // Little Endian
    assign write_data = (Jal || Jalr)? {34'b0, (mem_addr_I + 1)} : ((MemtoReg)? mem_rdata_D_BE : ALU_result);
    // assign write_data = (Jal || Jalr)? {32'b0, (mem_addr_I + 1), 2'b0} : ((MemtoReg)? mem_rdata_D_BE : ALU_result);

    assign reg_or_imm = (ALUSrc)? imm : read_data2;
    assign ALU_ctrl = (Branch)? 4'b1000 : ((MemRead || MemWrite)? 4'b0000 : ALUOp); // branch is SUB, ld/sd are ADD

    assign shifted_imm = imm << 1;

    // assign output
    assign mem_wen_D = MemWrite;
    assign mem_addr_D = ALU_result >> 2;
    assign mem_wdata_D = mem_wdata_D_LE;

    // Module Part
    Decoder decoder(
        rst_n,
        mem_rdata_I,
        type,
        format,
        ctrl,
        imm
    );
    Register register(
        clk,
        rst_n,
        RegWrite,
        read_reg1,
        read_reg2,
        write_reg,
        write_data,
        read_data1,
        read_data2
    );
    ALU alu(
        rst_n,
        read_data1,
        reg_or_imm,
        ALU_ctrl,
        ALU_result,
        zero
    );
    LE_CONV le0(
        mem_rdata_D,
        mem_rdata_D_BE
    );
    LE_CONV le1(
        read_data2,
        mem_wdata_D_LE
    );

    // Combinational Part (PC controller)
    always@ (*) begin
        if (type == BEQ) begin
            if (zero) begin
                next_mem_addr_I = mem_addr_I + (imm[29:0] >> 2);
            end
            else begin
                next_mem_addr_I = mem_addr_I + 1;
            end
        end
        else if (type == BNE) begin
            if (!zero) begin
                next_mem_addr_I = mem_addr_I + (imm[29:0] >> 2);
            end
            else begin
                next_mem_addr_I = mem_addr_I + 1;
            end
        end
        else if (Jal) begin
            next_mem_addr_I = mem_addr_I + (imm[29:0] >> 2);
        end
        else if (Jalr) begin
            next_mem_addr_I = read_data1 + (imm[29:0] >> 2);
        end
        else begin
            next_mem_addr_I = mem_addr_I + 1;
        end
    end
          
    // Sequential Part
    always@ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_addr_I <= 30'b0;
        end
        else begin
            mem_addr_I <= next_mem_addr_I;
        end
    end

endmodule

module Decoder( 
                rst_n,
                // for mem_I
                mem_rdata_I,
                // for result output
                instruction_type,
                instruction_format,
                ctrl_signal,
                immediate
                );

    // Input/Output Part
    input         rst_n;
    input  [31:0] mem_rdata_I;
	output [22:0] instruction_type;
	output [4:0]  instruction_format;
	output [11:0] ctrl_signal;
	output [63:0] immediate;

    // Parameter Part
    parameter R = 5'b10000;
    parameter I = 5'b01000;
    parameter S = 5'b00100;
    parameter B = 5'b00010;
    parameter J = 5'b00001;

    parameter JAL = {1'b1, 22'b0};
    parameter JALR = {1'b0, 1'b1, 21'b0};
    parameter BEQ = {2'b0, 1'b1, 20'b0}; 
    parameter BNE = {3'b0, 1'b1, 19'b0}; 
    parameter LD = {4'b0, 1'b1, 18'b0}; 
    parameter SD = {5'b0, 1'b1, 17'b0};  
    parameter ADDI = {6'b0, 1'b1, 16'b0};  
    parameter SLTI = {7'b0, 1'b1, 15'b0}; 
    parameter XORI = {8'b0, 1'b1, 14'b0}; 
    parameter ORI = {9'b0, 1'b1, 13'b0}; 
    parameter ANDI = {10'b0, 1'b1, 12'b0}; 
    parameter SLLI = {11'b0, 1'b1, 11'b0}; 
    parameter SRLI = {12'b0, 1'b1, 10'b0}; 
    parameter SRAI = {13'b0, 1'b1, 9'b0}; 
    parameter ADD = {14'b0, 1'b1, 8'b0}; 
    parameter SUB = {15'b0, 1'b1, 7'b0}; 
    parameter SLL = {16'b0, 1'b1, 6'b0}; 
    parameter SLT = {17'b0, 1'b1, 5'b0}; 
    parameter XOR = {18'b0, 1'b1, 4'b0}; 
    parameter SRL = {19'b0, 1'b1, 3'b0}; 
    parameter SRA = {20'b0, 1'b1, 2'b0}; 
    parameter OR = {21'b0, 1'b1, 1'b0}; 
    parameter AND = {22'b0, 1'b1};

    // Wire Part
    wire [6:0] opcode = mem_rdata_I[30:24];    // Little Endian
    wire [2:0] func3  = mem_rdata_I[22:20];    // Little Endian
    wire [6:0] func7  = mem_rdata_I[7:1];      // Little Endian

    // Reg Part
    reg [22:0] type;
    reg [4:0]  format;
    reg [11:0] ctrl;
    reg [63:0] imm;
    
    // Assign Part
    assign instruction_type = type;
    assign instruction_format = format;
    assign ctrl_signal = ctrl;
    assign immediate = imm;

    // Combinational Part
    always@ (*) begin
        case (opcode)
            7'b1101111: begin
                type = JAL;
                format = J;
                ctrl = 12'b100001111111;
                // Little Endian
                imm = { {43{mem_rdata_I[7]}}, {mem_rdata_I[11:8], mem_rdata_I[23:20]}, mem_rdata_I[12], {mem_rdata_I[6:0], mem_rdata_I[15:13]}, 1'b0 };
            end
            7'b1100111: begin
                type = JALR;
                format = I;
                ctrl = 12'b010001111111;
                // Little Endian
                imm = { {52{mem_rdata_I[7]}}, {mem_rdata_I[7:0], mem_rdata_I[15:12]} };
            end
            7'b1100011: begin
                format = B;
                // Little Endian
                imm = { {51{mem_rdata_I[7]}}, mem_rdata_I[7], mem_rdata_I[31], mem_rdata_I[6:1], mem_rdata_I[19:16], 1'b0 };
                case (func3)
                    3'b000: begin
                        type = BEQ;
                        ctrl = 12'b001001001111;
                    end
                    3'b001: begin
                        type = BNE;
                        ctrl = 12'b001001001111;
                    end
                    default: begin
                        type = 23'b0;
                        ctrl = 12'b0;
                    end
                endcase
            end
            7'b0000011: begin
                type = LD;
                format = I;
                ctrl =  12'b000101111111;
                // Little Endian
                imm = { {52{mem_rdata_I[7]}}, {mem_rdata_I[7:0], mem_rdata_I[15:12]} };
            end
            7'b0100011: begin
                type = SD;
                format = S;
                ctrl = 12'b000011011111;
                // Little Endian
                imm = { {52{mem_rdata_I[7]}}, mem_rdata_I[7:1], {mem_rdata_I[19:16], mem_rdata_I[31]}};
            end
            7'b0010011: begin
                format = I;
                // Little Endian
                imm = { {52{mem_rdata_I[7]}}, {mem_rdata_I[7:0], mem_rdata_I[15:12]} };
                case (func3)
                    3'b000: begin 
                        type = ADDI;
                        ctrl = 12'b000000110000;
                    end
                    3'b001: begin
                        type = SLLI;
                        ctrl = 12'b000000110001;
                    end
                    3'b010: begin
                        type = SLTI;
                        ctrl = 12'b000000110010;
                    end
                    3'b100: begin
                        type = XORI;
                        ctrl = 12'b000000110100;
                    end
                    3'b101: begin
                        case (func7)
                            7'b0000000: begin
                                type = SRLI;
                                ctrl = 12'b000000110101;
                            end
                            7'b0100000: begin
                                type = SRAI;
                                ctrl = 12'b000000111101;
                                imm = { {59{mem_rdata_I[0]}}, {mem_rdata_I[0], mem_rdata_I[15:12]} };
                            end
                            default: begin
                                type = 23'b0;
                                ctrl = 12'b0;
                            end
                        endcase
                    end
                    3'b110: begin
                        type = ORI;
                        ctrl = 12'b000000110110;
                    end
                    3'b111: begin
                        type = ANDI;
                        ctrl = 12'b000000110111;
                    end
                    default: begin
                        type = 23'b0;
                        ctrl = 12'b0;
                    end
                endcase
            end
            7'b0110011: begin
                format = R;
                imm = 64'b0;
                case (func3)
                    3'b000: begin
                        case (func7)
                            7'b0000000: begin
                                type = ADD;
                                ctrl = 12'b000000100000;
                            end
                            7'b0100000: begin
                                type = SUB;
                                ctrl = 12'b000000101000;
                            end
                            default: begin
                                type = 23'b0;
                                ctrl = 12'b0;
                            end
                        endcase
                    end 
                    3'b001: begin
                        type = SLL;
                        ctrl = 12'b000000100001;
                    end
                    3'b010: begin
                        type = SLT;
                        ctrl = 12'b000000100010;
                    end
                    3'b100: begin
                        type = XOR;
                        ctrl = 12'b000000100100;
                    end
                    3'b101: begin
                        case (func7)
                            7'b0000000: begin
                                type = SRL;
                                ctrl = 12'b000000100101;
                            end
                            7'b0100000: begin
                                type = SRA;
                                ctrl = 12'b000000101101;
                            end
                            default: begin
                                type = 23'b0;
                                ctrl = 12'b0;
                            end
                        endcase
                    end 
                    3'b110: begin
                        type = OR;
                        ctrl = 12'b000000100110;
                    end
                    3'b111: begin
                        type = AND;
                        ctrl = 12'b000000100111;
                    end
                    default: begin
                        type = 23'b0;
                        ctrl = 12'b0;
                    end
                endcase
            end
            default: begin
                format = 5'b0;
                imm = 64'b0;
                type = 23'b0;
                ctrl = 12'b0;
            end
        endcase
    end 

    // always@(negedge rst_n) begin
    //     type <= 0;
    //     format <= 0;
    //     ctrl <= 0;
    //     imm <= 0;
    // end

endmodule

module Register(
                clk,
                rst_n,
                RegWrite,
                read_reg1,
                read_reg2,
                write_reg,
                write_data,
                read_data1,
                read_data2
                );
    
    // Input/Output Part
    input           clk;
    input           rst_n;
    input           RegWrite;
    input  [4:0]    read_reg1, read_reg2, write_reg;
    input  [63:0]   write_data;
    output [63:0]   read_data1, read_data2;

    // Parameter Part
    parameter REGSIZE = 32;

    // Wire Part
    // Reg Part
    reg [63:0] register_r [0:REGSIZE-1];
    reg [63:0] register_w [0:REGSIZE-1];

    // Assign Part
    assign read_data1 = register_r[read_reg1];
    assign read_data2 = register_r[read_reg2];

    integer i;
    // Combinational Part
    always @(*) begin
        for (i=0; i<REGSIZE; i=i+1) begin
            register_w[i] = (RegWrite && (i == write_reg) && write_reg != 0) ? write_data : register_r[i];
        end
    end
    // Sequential Part
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<REGSIZE; i=i+1)
                register_r[i] <= 64'b0;
        end
        else begin
            for (i=0; i<REGSIZE; i=i+1)
                register_r[i] <= register_w[i];
        end
    end
endmodule

module ALU(
            rst_n,
            data1,
            data2,
            ctrl,  // ALU_ctrl
            result, // ALU_result
            zero
            );

    // Input/Output Part
    input         rst_n;
    input  [63:0] data1, data2;
    input  [3:0]  ctrl;
    output [63:0] result;
    output        zero; // boolean
    
    // Parameter Part
    localparam ADD = 4'b0000;
    localparam SUB = 4'b1000;
    localparam SLL = 4'b0001;
    localparam SLT = 4'b0010;
    localparam XOR = 4'b0100;
    localparam SRL = 4'b0101;
    localparam SRA = 4'b1101;
    localparam OR  = 4'b0110;
    localparam AND = 4'b0111;

    // Wire Part
    // Reg Part
    reg [63:0] res;
    // Assign Part 
    assign result = res;
    assign zero = !result;
    // Combinational Part
    always@(*) begin
        case(ctrl)
            ADD: res = (data1 + data2);
            SUB: res = (data1 - data2);
            SLL: res = (data1 << data2);
            SLT: res = (data1 < data2);
            XOR: res = (data1 ^ data2);
            SRL: res = (data1 >> data2);
            SRA: res = (data1 >>> data2);
            OR:  res = (data1 | data2);
            AND: res = (data1 & data2);
            default: res = 64'b0;
        endcase
    end

endmodule

// little endian converter
module LE_CONV(
        in,
        out
        );
        input  [63:0] in;
        output [63:0] out;

        wire   [63:0] tmp;

        assign tmp[7:0]   = in[63:56];
        assign tmp[15:8]  = in[55:48];
        assign tmp[23:16] = in[47:40];
        assign tmp[31:24] = in[39:32];
        assign tmp[39:32] = in[31:24];
        assign tmp[47:40] = in[23:16];
        assign tmp[55:48] = in[15:8];
        assign tmp[63:56] = in[7:0];
        assign out = tmp;

endmodule