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
            mem_rdata_I);

    input              clk, rst_n;   // Clock signal
    input       [31:0] mem_rdata_I;  // Active low asyncrhonous reset signal
    output reg  [29:0] mem_addr_I;   // Output address of instruction memory
    input       [31:0] mem_rdata_I;  // Instruction read from instruction memory
    output reg         mem_wen_D;    // Write-enable
    output reg  [63:0] mem_wdata_D;  // Data store to data memory
    input       [63:0] mem_rdata_D;  // Data read from data memory
    
    // Wire Part
    wire [6:0] opcode = mem_rdata_I[6:0];
    wire [2:0] func3  = mem_rdata_I[14:12];
    wire [6:0] func7  = mem_rdata_I[31:25];

    // Reg Part
    reg [29:0]  next_mem_addr_I;
    reg         next_mem_wen_D;
    reg [63:0]  next_mem_wdata_D;

    // reg [31:0] instruction;
    reg [22:0] instruction_type;    // JAL, JALR ... 23 bits
	reg [ 4:0] instruction_format;  // R, I, S, B, J. 5 bits
    reg [11:0] ctrl_signal;         // control signal
    reg [31:0] immediate;           // immediate value

    // Paramter Part
    parameter JAL  = {1'b1, 22'b0}      ;       // 1101111
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

    parameter R = 5'b10000;
    parameter I = 5'b01000;
    parameter S = 5'b00100;
    parameter B = 5'b00010;
    parameter J = 5'b00001;

    // Combinational Part
    always@ (*)
    begin
        // instruction decoder (HW3)
        case (opcode)
            7'b1101111: begin
                instruction_type = JAL;
                instruction_format = J;
                ctrl_signal =  12'b100001111111;
            end
            7'b1100111: begin
                instruction_type = JALR;
                instruction_format = I;
                ctrl_signal =  12'b010001111111;
            end
            7'b1100011: begin
                instruction_format = B;
                case (func3)
                    3'b000: begin
                        instruction_type = BEQ;
                        ctrl_signal = 12'b001001001111;
                    end
                    3'b001: begin
                        instruction_type = BNE;
                        ctrl_signal =  12'b001001001111;
                    end
                endcase
            end
            7'b0000011: begin
                instruction_type = LD;
                instruction_format = I;
                ctrl_signal =  12'b000101111111;
            end
            7'b0100011: begin
                instruction_type = SD;
                instruction_format = S;
                ctrl_signal =  12'b000011011111;
            end
            7'b0010011: begin
                instruction_format = I;
                case (func3)
                    3'b000: begin 
                        instruction_type = ADDI;
                        ctrl_signal =  12'b000000110000;
                    end
                    3'b001: begin
                        instruction_type = SLLI;
                        ctrl_signal =  12'b000000110001;
                    end
                    3'b010: begin
                        instruction_type = SLTI;
                        ctrl_signal =  12'b000000110010;
                    end
                    3'b100: begin
                        instruction_type = XORI;
                        ctrl_signal =  12'b000000110100;
                    end
                    3'b101: begin
                        case (func7)
                            7'b0000000: begin
                                instruction_type = SRLI;
                                ctrl_signal =  12'b000000110101;
                            end
                            7'b0100000: begin
                                instruction_type = SRAI;
                                ctrl_signal =  12'b000000111101;
                            end
                        endcase
                    end
                    3'b110: begin
                        instruction_type = ORI;
                        ctrl_signal =  12'b000000110110;
                    end
                    3'b111: begin
                        instruction_type = ANDI;
                        ctrl_signal =  12'b000000110111;
                    end
                endcase
            end
            7'b0110011: begin
                instruction_format = R;
                case (func3)
                    3'b000: begin
                        case (func7)
                            7'b0000000: begin
                                instruction_type = ADD;
                                ctrl_signal =  12'b000000100000;
                            end
                            7'b0100000: begin
                                instruction_type = SUB;
                                ctrl_signal =  12'b000000101000;
                            end
                        endcase
                    end 
                    3'b001: begin
                        instruction_type = SLL;
                        ctrl_signal =  12'b000000100001;
                    end
                    3'b010: begin
                        instruction_type = SLT;
                        ctrl_signal =  12'b000000100010;
                    end
                    3'b100: begin
                        instruction_type = XOR;
                        ctrl_signal =  12'b000000100100;
                    end
                    3'b101: begin
                        case (func7)
                            7'b0000000: begin
                                instruction_type = SRL;
                                ctrl_signal =  12'b000000100101;
                            end
                            7'b0100000: begin
                                instruction_type = SRA;
                                ctrl_signal =  12'b000000101101;
                            end
                        endcase
                    end 
                    3'b110: begin
                        begininstruction_type = OR;
                        ctrl_signal =  12'b000000100110;
                    end
                    3'b111: begin
                        instruction_type = AND;
                        ctrl_signal =  12'b000000100111;
                    end
                endcase
            end

        endcase
        
		// case (instruction_format) // for immediate
		// 	I: immediate = { {20{instruction[31]}}, instruction[31:20] };
		// 	S: immediate = { {20{instruction[31]}}, instruction[31:25], instruction[11:7] };
		// 	B: immediate = { {19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0 };
		// 	J: immediate = { {11{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0 };
		// 	default: immediate = 32'b0;
		// endcase

    end
    
    // Sequential Part
    always@ (posedge clk or negedge rst_n)
    begin
        if (!rst_n) 
        begin
            mem_addr_I  <= 30'b0;
            mem_wen_D   <= 1'b0;
            mem_wdata_D <= 64'b0;
        end
        else begin
            mem_addr_I <= next_mem_addr_I;
            mem_wen_D <= next_mem_wen_D;
            mem_wdata_D <= next_mem_wdata_D;
        end

    end

endmodule

// Decoder
module decoder( mem_rdata_I,
                // for result output
                instruction_type,
                instruction_format,
                ctrl_signal,
                immediate
                );

endmodule