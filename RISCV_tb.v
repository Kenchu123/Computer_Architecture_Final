// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 6 // You can modify your clock frequency
`define END_CYCLE 50 // You can modify your maximum cycles

`include "memory.v"
`ifdef tb1
	`define DMEM_DATA "./pattern/data_1.txt"
	`define DMEM_ANS  "./pattern/ans_1.txt"
	`define IMEM_INIT "./pattern/inst_RV64I_1.txt"
`elsif tb2
	`define DMEM_DATA "./pattern/data_2.txt"
	`define DMEM_ANS  "./pattern/ans_2.txt"
	`define IMEM_INIT "./pattern/inst_RV64I_2.txt"
`elsif tb3
	`define DMEM_DATA "./pattern/data_3.txt"
	`define DMEM_ANS  "./pattern/ans_3.txt"
	`define IMEM_INIT "./pattern/inst_RV64I_3.txt"
`endif


`ifdef RTL
	`include "RISCV.v"
`endif
`ifdef SYN
	`include "./Netlist/RISCV_syn.v"
	`include "tsmc13.v"
	`define SDF
	`define SDFFILE "./Netlist/RISCV_syn.sdf"
`endif


module RISCV_tb;

    reg         clk, rst_n ;
    
    wire        mem_wen_D  ;
    wire [31:2] mem_addr_D ;
    wire [63:0] mem_wdata_D;
    wire [63:0] mem_rdata_D;
    
    wire [31:2] mem_addr_I ;
    wire [31:0] mem_rdata_I;
    
    reg  [63:0] mem_data_ans [0:255];

    integer i;
    
    integer eof;
    reg eof_find;

    integer error_num;
    
    RISCV chip0(
        clk,
        rst_n,
        // for mem_D
        mem_wen_D,
        mem_addr_D,
        mem_wdata_D,
        mem_rdata_D,
        // for mem_I
        mem_addr_I,
        mem_rdata_I);
    
    memory_I mem_I(
        .clk(clk),
        .wen(1'b0),
        .a(mem_addr_I[9:2]),
        .d(32'd0),
        .q(mem_rdata_I));
    
    memory_D mem_D(
        .clk(clk),
        .wen(mem_wen_D),
        .a(mem_addr_D[10:3]),
        .d(mem_wdata_D),
        .q(mem_rdata_D));
       
    `ifdef SDF
        initial $sdf_annotate(`SDFFILE, chip0);
    `endif
    
    // Initialize the data memory
    initial begin
        $fsdbDumpfile("RISCV.fsdb");            
        $fsdbDumpvars(0,RISCV_tb,"+mda");

        $display("------------------------------------------------------------\n");
        $display("START!!! Simulation Start .....\n");
        $display("------------------------------------------------------------\n");
        
        clk = 1;
        rst_n = 1'b1;
        #(`CYCLE*0.1) rst_n = 1'b0;
        #(`CYCLE*2.0) rst_n = 1'b1;
        
        for (i=0; i<256; i=i+1) mem_D.mem[i]    = 64'h00_00_00_00_00_00_00_00; // reset data in mem_D
        $readmemh (`DMEM_DATA, mem_D.mem);                        			   // initialize data in mem_D
        for (i=0; i<256; i=i+1) mem_data_ans[i] = 64'h00_00_00_00_00_00_00_00;
        $readmemh (`DMEM_ANS , mem_data_ans);                     			   // answer lists
        $readmemh (`IMEM_INIT, mem_I.mem);                        			   // initialize data in mem_I
        eof_find = 0;
        for (i=0; i<256; i=i+1) begin
            if (mem_I.mem[i] === 32'bx) begin
                if (eof_find == 0) begin
                    eof_find = 1;
                    eof = i;
                end
                mem_I.mem[i] = 32'h33_00_00_00;
            end
        end

        #(`CYCLE*`END_CYCLE)
        $display("============================================================\n");
        $display("Simulation time is longer than expected.");
        $display("The test result is .....FAIL :(\n");
        $display("============================================================\n");
        $finish;
    end

    always @(negedge clk) begin
        if (mem_addr_I >= eof) begin
            error_num = 0;
            for (i=0; i<256; i=i+1) begin
                if (mem_D.mem[i] !== mem_data_ans[i]) begin
                    if (error_num == 0)
                        $display("Error!");
                    error_num = error_num + 1;
                    $display("  Addr = 0x%2h  Correct ans: 0x%h  Your ans: 0x%h", 8*i, mem_data_ans[i], mem_D.mem[i]);
                end
            end
            if (error_num > 0) begin
                $display(" ");
                $display("================================================================\n");
                $display("There are total %4d errors in the data memory", error_num);
                $display("The test result is .....FAIL :(\n");
                $display("================================================================\n");
            end
            else begin
			`ifdef RTL
				`ifdef tb1
				$display("===============The RTL result for tb1 is PASS===============");
				$display("         |");
				$display("        -+-");
				$display("         A");
				$display("        /=\\               /\\  /\\    ___  _ __  _ __ __    __");
				$display("      i/ O \\i            /  \\/  \\  / _ \\| '__|| '__|\\ \\  / /");
				$display("      /=====\\           / /\\  /\\ \\|  __/| |   | |    \\ \\/ /");
				$display("      /  i  \\           \\ \\ \\/ / / \\___/|_|   |_|     \\  /");
				$display("    i/ O * O \\i                                       / /");
				$display("    /=========\\        __  __                        /_/    _");
				$display("    /  *   *  \\        \\ \\/ /        /\\  /\\    __ _  ____  | |");
				$display("  i/ O   i   O \\i       \\  /   __   /  \\/  \\  / _` |/ ___\\ |_|");
				$display("  /=============\\       /  \\  |__| / /\\  /\\ \\| (_| |\\___ \\  _ ");
				$display("  /  O   i   O  \\      /_/\\_\\      \\ \\ \\/ / / \\__,_|\\____/ |_|");
				$display("i/ *   O   O   * \\i");
				$display("/=================\\");
				$display("       |___|\n");
				`elsif tb2
				$display("===============The RTL result for tb2 is PASS===============");
				$display("                                                 *       ");
				$display("                                                         ");
				$display("               *        /)___________/)                  ");
				$display("                       / ,--o ______/ /       *         *");
				$display("                      / /__\\       / |               *  ");
				$display("   *                 /  {''}]     /  |                   ");
				$display("                     .--{~`/--.   )  \\__              * ");
				$display("                    /   { }    \\ /     /                ");
				$display("         *         /_/   ~   /_//'  / /                  ");
				$display("                 .-\"\"===\"===\"\" |   / /              ");
				$display("                /  |-(__)(__)/__| /_/                    ");
				$display("               /   | \\  |\\  |__ ) /                    ");
				$display("              /   / //_/ /_/   / /                       ");
				$display("             /  _/_/________  / /                        ");
				$display("            /  (            (/ /                         ");
				$display("           /    \\==========   /                         ");
				$display("       snd/      (___________/                           ");
				$display("         /      _/ /     _/ /                            ");
				$display(" |\\/|   /      \\\\_/     \\\\_/                        ");
				$display(" 00 | _/________ |\\/|    /                              ");
				$display("/_/|_\\/          00 |  _/                               ");
				$display(" __/ )|         /_/|_\\//                                ");
				$display("VV--   \\         __/ )|                                 ");
				$display("   |_   |       VV--   \\                                ");
				$display("  / / / )          |_   |                                ");
				$display(" |_|_/\\_ \\____    / / / )                              ");
				$display("  ////  '-----`  |_|_/\\_ \\____                         ");
				$display(" ////             ////  '-----`                          ");
				$display(" \" \"             ////                                  ");
				$display("                 \" \"                                 \n");
				`elsif tb3
				$display("====  The RTL result for negative number is correct  ====");
				$display("====Make sure to generate more patterns for hidden tb====\n");
				`endif
			`elsif SYN
				`ifdef tb1
				$display("===============The SYN result for tb1 is PASS===============");
				$display(".;;,     .;;,                                                     ");
				$display("`  ;;   ;;  '                                                     ");
				$display("   ;;   ;; ,  .;;;.   .;;,;;;,  .;;,;;;,  .;;.  .;;.              ");
				$display(" ,;;;;;;;;;'  `   ;;  ` ;;   ;; ` ;;   ;; ` ;;  ;; '              ");
				$display(" ` ;;   ;;    .;;.;;    ;;   ;;   ;;   ;;   ;;  ;;                ");
				$display("   ;;   ;;    ;;  ;; ,  ;;   ;;   ;;   ;;   ;;  ;;                ");
				$display(".  ;;    ';;' `;;;';;'  ;;';;'    ;;';;'     `;;';                ");
				$display("';;'                    ;;        ;;            ;;                ");
				$display("                     .  ;;     .  ;;         .  ;;                ");
				$display("                     ';;'      ';;'          ';;'                 ");
				$display("                                                                  ");
				$display("               .;;, ,;;;,                                         ");
				$display("               `  ;;    ;;                                        ");
				$display("                  ;;    ;;     ,;;,  .;;.      .;;,               ");
				$display("                  ;;    ;;    ;;  ;; ` ;;      ;; '               ");
				$display("                  ;;    ;;    ;;;;;'   ;;  ;;  ;;                 ");
				$display("                  ;;    ;;    ;;   .   ;;  ;;  ;;                 ");
				$display("               .  ;;     ';;'  `;;;'    `;;'`;;'                  ");
				$display("               ';;'                                               ");
				$display("                                                                  ");
				$display("                              .;;.     .;;.                       ");
				$display("                              `  ;;   ;;  '                       ");
				$display("                                 ;;   ;;   .;;,  .;;;.   .;;.;;;, ");
				$display("                                 ;;   ;;  ;;  ;; '   ;;  ` ;;   ' ");
				$display("                                 ;;   ;;  ;;;;;' .;;,;;    ;;     ");
				$display("                                  `;;;';  ;;   . ;;  ;; ,  ;;     ");
				$display("                                      ;;   `;;;'  `;;';;'  ;'     ");
				$display("                                      ;;                          ");
				$display("                                  .'  ;;                          ");
				$display("                                  ';;;'                           ");
				`elsif tb2
				$display("===============The SYN result for tb2 is PASS===============");
				$display(" _____________________ ");
				$display("|  _________________  |");
				$display("| | CA Final:  100  | |");
				$display("| | GPA:      4.30  | |");
				$display("| |_________________| |");
				$display("|  ___ ___ ___   ___  |");
				$display("| | 7 | 8 | 9 | | + | |");
				$display("| |___|___|___| |___| |");
				$display("| | 4 | 5 | 6 | | - | |");
				$display("| |___|___|___| |___| |");
				$display("| | 1 | 2 | 3 | | x | |");
				$display("| |___|___|___| |___| |");
				$display("| | . | 0 | = | | / | |");
				$display("| |___|___|___| |___| |");
				$display("|_____________________|\n");
				`elsif tb3
				$display("====  The SYN result for negative number is correct  ====");
				$display("====Make sure to generate more patterns for hidden tb====\n");
				`endif
			`endif
			end
            $finish;
        end
    end
        
    always #(`CYCLE*0.5) clk = ~clk;
        
endmodule
