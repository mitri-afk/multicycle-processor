// mircoprocessor.sv
// Dimitri Avila
// davila@hmc.edu
// May 9th, 2023
// ~*~*~*~*~*~*~**~

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

///////////////////////////////////////////////////////////////
// testbench
//
// Expect simulator to print "Simulation succeeded"
// when the value 27 (0x19) is written to address 100 (0x64)
///////////////////////////////////////////////////////////////

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [31:0] hash;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      hash <= 0;
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 27) begin // ADDED 27 (changed it from 25 to 27, since 27 should be written to mem[100])
          $display("Simulation succeeded");
 	   	  $display("hash = %h", hash);
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end

  // Make 32-bit hash of instruction, PC, ALU
  always @(negedge clk)
    if (~reset) begin
      hash = hash ^ dut.rvmulti.dp.Instr ^ dut.rvmulti.dp.PC;
      if (MemWrite) hash = hash ^ WriteData;
      hash = {hash[30:0], hash[9] ^ hash[29] ^ hash[30] ^ hash[31]};
    end
endmodule

///////////////////////////////////////////////////////////////
// top
//
// Instantiates multicycle RISC-V processor and memory
///////////////////////////////////////////////////////////////

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] ReadData;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, 
                     WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

///////////////////////////////////////////////////////////////
// mem
//
// Single-ported RAM with read and write ports
// Initialized with machine language program
///////////////////////////////////////////////////////////////

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////////////////////////////
// riscvmulti
//
// Multicycle RISC-V microprocessor
///////////////////////////////////////////////////////////////

module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,
                  input  logic [31:0] ReadData);


// Instantiate controller and datapath...
// Variables: (the ones not included in this module, but we need to pass into controller and datapath)
logic [31:0] Instr;
logic Zero, AdrSrc, IRWrite, PCWrite, RegWrite;
logic [1:0] ImmSrc, ALUSrcA, ALUSrcB, ResultSrc;
logic [2:0] ALUControl;

opcodetype op;
assign op = opcodetype'(Instr[6:0]);

controller c(clk, reset, op, Instr[14:12], 
	     Instr[30], Zero, ImmSrc, ALUSrcA, ALUSrcB, 
	     ResultSrc, AdrSrc, ALUControl, IRWrite, 
	     PCWrite, RegWrite, MemWrite);

datapath dp(clk, reset, RegWrite, ReadData, 
	    ImmSrc, ALUSrcA, ALUSrcB, ALUControl, 
	    ResultSrc, AdrSrc, IRWrite, PCWrite, 
	    Instr, WriteData, Adr, Zero);
endmodule


///////////////////////////////////////////////////////////////
// Processor modules...
///////////////////////////////////////////////////////////////

module controller(input  logic       clk,
                  input  logic       reset,  
                  input  opcodetype  op,	// [6:0]   Instr
                  input  logic [2:0] funct3,    // [14:12] Instr
                  input  logic       funct7b5,  // [30]    Instr
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);

logic Branch, PCUpdate, and1;
logic [1:0] ALUOp;

mainFSM    myMain  (clk, reset, op, Branch, PCUpdate, RegWrite, MemWrite, IRWrite, ResultSrc, ALUSrcA, ALUSrcB, AdrSrc, ALUOp);
aludecoder aluDecoder  (ALUOp, funct3, op[5], funct7b5, ALUControl);
IDecoder   iDecoder(op, ImmSrc);

and g1(and1, Zero, Branch);
or  g2(PCWrite, and1, PCUpdate);
endmodule



// Datapath...
module datapath(input  logic        clk, reset,
		input  logic        RegWrite,
		input  logic [31:0] ReadData,
		input  logic [1:0]  ImmSrc,
		input  logic [1:0]  ALUSrcA, ALUSrcB,
		input  logic [2:0]  ALUControl,
		input  logic [1:0]  ResultSrc,
		input  logic 	    AdrSrc,
		input  logic        IRWrite,
		input  logic        PCWrite,

		output logic [31:0] Instr,
		output logic [31:0] WriteData,
		output logic [31:0] Adr,
		output logic 	    Zero);

logic [31:0] PC, OldPC, Data;
logic [31:0] SrcA, SrcB;
logic [31:0] rd1, rd2, A;
logic [31:0] ImmExt;
logic [31:0] Result; // (aka PCNext)
logic [31:0] ALUOut, ALUResult;


// Flop+Enable w/ Result(aka PCNext) -> PC	**PCNext is the SAME as Result, so I removed PCNext.
flopenr flop1(clk, reset, PCWrite, Result, PC);

// Mux2 w/ PC or Result(aka PCNext) -> Adr	**PCNext is the SAME as Result, so I removed PCNext.
mux2 #(32) pcmux(PC, Result, AdrSrc, Adr);

// Flop+Enable w/ PC -> OldPC
flopenr flop2(clk, reset, IRWrite, PC, OldPC);

// Flop+Enable w/ ReadData -> Instr
flopenr flop3(clk, reset, IRWrite, ReadData, Instr);

// Flop w/ ReadData -> Data
flopr flop4 (clk, reset, ReadData, Data);

// Mux3 w/ (PC, OldPC, and A) -> SrcA
mux3 #(32) srcamux(PC, OldPC, A, ALUSrcA, SrcA);

// Mux3 w/ (WriteData, ImmExt, 4) -> SrcB
mux3 #(32) srcbmux(WriteData, ImmExt, 32'd4, ALUSrcB, SrcB); // one input is 4. make sure it's 32'd4 and not 32'b4 (at least according to riscvsingle).

// ALU with SrcA & SrcB as inputs
alu myalu(SrcA, SrcB, ALUControl, ALUResult, Zero); // alu module definition includes zero as the last parameter?

// Register file logic
regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, rd1, rd2);
extend      ext(Instr[31:7], ImmSrc, ImmExt);

// Flop w/ rd1 -> A
flopr flop6(clk, reset, rd1, A);

// Flop w/ rd2 -> WriteData
flopr flop7(clk, reset, rd2, WriteData);

// Flop w/ ALUResult -> ALUOut
flopr flop8(clk, reset, ALUResult, ALUOut);

// Mux3 w/ (ALUOut, Data, and ALUResult) -> Result
mux3 #(32) resultmux(ALUOut, Data, ALUResult, ResultSrc, Result);
endmodule

// Main FSM
module mainFSM(input  logic       clk,
	       input  logic       reset,
	       input  logic [6:0] op,
	       output logic       Branch,
	       output logic       PCUpdate,
               output logic       RegWrite,
               output logic       MemWrite,
               output logic       IRWrite,
               output logic [1:0] ResultSrc,
               output logic [1:0] ALUSrcA, 
	       output logic [1:0] ALUSrcB,
               output logic       AdrSrc,
	       output logic [1:0] ALUOp);

typedef enum logic [6:0] {S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10} statetype;
statetype state, nextstate;

// state register
always_ff @ (posedge clk, posedge reset)
if (reset) state <= S0;
else 	   state <= nextstate;

// next state logic
always_comb
  case (state)
     S0:           	                    nextstate = S1;
     S1: if (op == lw_op | op == sw_op)     nextstate = S2;
         else if (op == r_type_op)          nextstate = S6;
         else if (op == i_type_alu_op)      nextstate = S8;
         else if (op == jal_op) 	    nextstate = S9;
         else if (op == beq_op)             nextstate = S10;
     S2: if (op == lw_op)                   nextstate = S3;
         else if (op == sw_op)              nextstate = S5;
     S3: 				    nextstate = S4;
     S4: 				    nextstate = S0;
     S5: 				    nextstate = S0;
     S6: 				    nextstate = S7;
     S7:				    nextstate = S0;
     S8:				    nextstate = S7;
     S9: 				    nextstate = S7;
     S10:				    nextstate = S0;
     default: 				    nextstate = S0; 
endcase

// output logic
assign ALUSrcA[0]   = (state == S1 | state == S9);
assign ALUSrcA[1]   = (state == S2 | state == S6 | state == S8 | state == S10);
assign ALUSrcB[0]   = (state == S1 | state == S2 | state == S8);
assign ALUSrcB[1]   = (state == S0 | state == S9);
assign ALUOp[0]     = (state == S10);
assign ALUOp[1]     = (state == S6 | state == S8);
assign AdrSrc       = (state == S3 | state == S5);
assign IRWrite      = (state == S0);
assign ResultSrc[0] = (state == S4);
assign ResultSrc[1] = (state == S0);
assign Branch       = (state == S10);
assign PCUpdate     = (state == S0 | state == S9);
assign MemWrite     = (state == S5);
assign RegWrite     = (state == S4 | state == S7);
endmodule
		


// ALU Decoder...
module aludecoder(input  logic [1:0] ALUOp, // s1,s0
                  input  logic [2:0] funct3, //f2, f1, f0
                  input  logic op_5, funct7_5, // a,b
                  output logic [2:0] ALUControl); // s'2, s'1, s'0

  logic  RtypeSub;
  assign RtypeSub = funct7_5 & op_5;  // TRUE for R-type subtract instruction


always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if(RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
		 3'b001:    ALUControl = 3'b111; // bseti (ADDED)
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
		 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule

// Instruction Decoder
module IDecoder(input logic [6:0] op,
		output logic [1:0] ImmSrc);
always_comb
   casez (op)
      lw_op:         ImmSrc = 2'b00;
      sw_op:         ImmSrc = 2'b01;
      r_type_op:     ImmSrc = 2'b00;
      beq_op:        ImmSrc = 2'b10;
      i_type_alu_op: ImmSrc = 2'b00;
      jal_op:        ImmSrc = 2'b11;
      default	     ImmSrc = 2'b00;
   endcase
endmodule

// Resettable flip-flop
module flopr (input   logic clk, reset,
              input  logic [31:0] d, 
              output logic [31:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 32'b0;
    else       q <= d;
endmodule

// 2 input mutliplexer
module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

// 3 input multiplexer
module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

// ALU
module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum, bseti_b;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign bseti_b = 1 << b;

  always_comb
    case (alucontrol)
      3'b000:  result = sum;       // add
      3'b001:  result = sum;       // subtract
      3'b010:  result = a & b;     // and
      3'b011:  result = a | b;     // or
      3'b101:  result = sum[31];   // slt
      3'b111:  result = a | (1 << b[4:0]); //bseti (ADDED)
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
endmodule


// enabled flip-flop with reset
module flopenr(input  logic       clk, reset,
	       input  logic       en, 
	       input  logic [31:0] d,
	       output logic [31:0] q);

// asynchronous reset
always_ff @ (posedge clk, posedge reset)
	if (reset) q <= 32'b0;
	else if (en) q <= d;
endmodule


module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule


// Extend module...
module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
      default: immext = 32'bx; // undefined
    endcase             
endmodule

// Flip-Flop...
module flop(input   logic       clk,
            input   logic [31:0] d,  
            output  logic [31:0] q);

  always_ff @(posedge clk)        
     q <= d; 
endmodule
