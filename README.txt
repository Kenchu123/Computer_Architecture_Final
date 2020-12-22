Update: 2020/12/14
This testbench supports RV64I, RTL, and SYN

============ Files Description ==============

  Netlist/			-- For synthesis output files
  pattern/			-- RTL&SYN pattern files
    - data_1.txt/data_2.txt/data_3.txt		-- Memory data file
    - ans_1.txt/ans_2.txt/ans_3.txt			-- Expected memory answer
    - inst_RV64I_1.txt/inst_RV64I_2.txt
	  inst_RV64I_2.txt						-- Instruction sets
	- patter_gen_1.py						-- python code to generate different input for tb1
											   data_.txt and ans_1.txt will generate directly
	- patter_gen_2.s						-- Assembly code for inst_RV64I_2 (Try to generate different tb)
	- patter_gen_3.s						-- Assembly code for inst_RV64I_3 (Try to generate different tb)
  Report/			-- For synthesis output report files
  .cshrc			-- enovironment file for csh mode(Put it at the top folder of your work station)
  .synopsys_dc		-- file for synthesis
  RISCV.v			-- Your HW main file
  RISCV_syn.sdc		-- Constraints for synthesis
  decoder_tb		-- Testbench for RTL and Synthesis
  run.tcl			-- tcl file for synthesis
  README

============ Environment Setting ==============
Source (environment in NTU):
    >	source /usr/cad/cadence/cshrc
    >	source /usr/cad/synopsys/CIC/verdi.cshrc
    >	source /usr/cad/synopsys/CIC/synthesis.cshrc

	or put the .cshrc file in your top folder ./b07xxx/.cshrc, it will source automatically next time you log in.

RTL simulation:
    >	ncverilog RISCV_tb.v +access+r +define+RTL+tb1
	>	ncverilog RISCV_tb.v +access+r +define+RTL+tb2
	>	ncverilog RISCV_tb.v +access+r +define+RTL+tb3
    
--------------------------------------------------------------------------
Synthesis command:
- Open Design Compiler:
    > dc_shell
- In Design Compiler:
    dc_shell> source run.tcl
	or run the tcl file line by line to prevent setting error

- Check if your design passes timing slack:
    dc_shell> report_timing
- Check area:
    dc_shell> report_area
- Close Design Compiler:
    dc_shell> exit
    
--------------------------------------------------------------------------
Post-synthesis simulation:
- Check if you have a SDF file (RISCV_syn.sdf)
- Check if you have a library file (tsmc13.v)
- Note: To copy tsmc13.v to your current directory (environment in NTU):
    >	cp /home/raid7_2/course/cvsd/CBDK_IC_Contest/CIC/Verilog/tsmc13.v .

- Run:
    >	ncverilog RISCV_tb.v +access+r +define+SYN+tb1
	>	ncverilog RISCV_tb.v +access+r +define+SYN+tb2
	>	ncverilog RISCV_tb.v +access+r +define+SYN+tb3