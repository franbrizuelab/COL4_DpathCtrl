//=========================================================================
// 5-Stage RISCV Datapath
//=========================================================================

`ifndef RISCV_CORE_DPATH_V
`define RISCV_CORE_DPATH_V

`include "riscvpipeline-InstMsg.v"
`include "riscvpipeline-CoreDpathAlu.v"
`include "riscvpipeline-CoreDpathMulDiv.v"
`include "riscvpipeline-CoreDpathRegfile.v"

module riscv_CoreDpath
(
  input clk,
  input reset,

  // Instruction Memory Port

  output [31:0] imemreq_msg_addr,

  // Data Memory Port

  output [31:0] dmemreq_msg_addr,
  output [31:0] dmemreq_msg_data,
  input  [31:0] dmemresp_msg_data,

  // Controls Signals (ctrl->dpath)

  // NOTE: Change the name of the original varible. 
  // It will serve the same purpose, but keeping the name
  // Will lead to confusion

  //output     [ 1:0] pc_mux_sel_Phl,
  input   [1:0] brj_mux_sel_Xhl,
  input   [1:0] op0_mux_sel_Dhl,
  input   [2:0] op1_mux_sel_Dhl,
  input  [31:0] inst_Dhl,
  input   [3:0] alu_fn_Xhl,
  input   [2:0] muldivreq_msg_fn_Xhl,
  input         muldivreq_val,
  output        muldivreq_rdy,
  output        muldivresp_val,
  input         muldivresp_rdy,
  input         muldiv_mux_sel_Xhl,
  input         execute_mux_sel_Xhl,
  input   [2:0] dmemresp_mux_sel_Mhl,
  input         dmemresp_queue_en_Mhl,
  input         dmemresp_queue_val_Mhl,
  input         wb_mux_sel_Mhl,
  input         rf_wen_Whl,
  input  [ 4:0] rf_waddr_Whl,
  input         stall_Fhl,
  input         stall_Dhl,
  input         stall_Xhl,
  input         stall_Mhl,
  input         stall_Whl,

  // DISCLAIMER: Connections to handle branching info between stages
  input brj_taken_Xhl,

  // DISCLAIMER: connections added to implement imemreq_val
  output pc_mux_out_valid_Phl,


  // Control Signals (dpath->ctrl)

  output        branch_cond_eq_Xhl,
  output        branch_cond_ne_Xhl,
  output        branch_cond_lt_Xhl,
  output        branch_cond_ltu_Xhl,
  output        branch_cond_ge_Xhl,
  output        branch_cond_geu_Xhl,
  output [31:0] proc2csr_data_Whl
);


  // LOCALPARAM DEFINITION
  // Generic Parameters

  localparam n = 1'd0;
  localparam y = 1'd1;

  // Register specifiers

  localparam rx = 5'bx;
  localparam r0 = 5'd0;

  // Branch Type

  localparam br_x    = 3'bx;
  localparam br_none = 3'd0;
  localparam br_beq  = 3'd1;
  localparam br_bne  = 3'd2;
  localparam br_blt  = 3'd3;
  localparam br_bltu = 3'd4;
  localparam br_bge  = 3'd5;
  localparam br_bgeu = 3'd6;

  // BRJ MUX SELECT

  localparam pm_x   = 2'bx;  // Don't care
  localparam pm_p   = 2'd0;  // Use pc+4
  localparam pm_b   = 2'd1;  // Use branch address
  localparam pm_j   = 2'd2;  // Use jump address
  localparam pm_r   = 2'd3;  // Use jump register

  // PC MUX SELECT

  localparam pc_brj   = 1'd0;  // Use Output of brj MUX
  localparam pc_pcplus4 = 1'd0;  // Use PcPlus4_Phl

  // Operand 0 Mux Select

  localparam am_x     = 2'bx;
  localparam am_rdat  = 2'd0; // Use output of bypass mux for rs1
  localparam am_pc    = 2'd1; // Use current PC
  localparam am_pc4   = 2'd2; // Use PC + 4
  localparam am_0     = 2'd3; // Use constant 0

  // Operand 1 Mux Select

  localparam bm_x      = 3'bx; // Don't care
  localparam bm_rdat   = 3'd0; // Use output of bypass mux for rs2
  localparam bm_shamt  = 3'd1; // Use shift amount
  localparam bm_imm_u  = 3'd2; // Use U-type immediate
  localparam bm_imm_sb = 3'd3; // Use SB-type immediate
  localparam bm_imm_i  = 3'd4; // Use I-type immediate
  localparam bm_imm_s  = 3'd5; // Use S-type immediate
  localparam bm_0      = 3'd6; // Use constant 0

  // ALU Function

  localparam alu_x    = 4'bx;
  localparam alu_add  = 4'd0;
  localparam alu_sub  = 4'd1;
  localparam alu_sll  = 4'd2;
  localparam alu_or   = 4'd3;
  localparam alu_lt   = 4'd4;
  localparam alu_ltu  = 4'd5;
  localparam alu_and  = 4'd6;
  localparam alu_xor  = 4'd7;
  localparam alu_nor  = 4'd8;
  localparam alu_srl  = 4'd9;
  localparam alu_sra  = 4'd10;

  // Muldiv Function

  localparam md_x    = 3'bx;
  localparam md_mul  = 3'd0;
  localparam md_div  = 3'd1;
  localparam md_divu = 3'd2;
  localparam md_rem  = 3'd3;
  localparam md_remu = 3'd4;

  // MulDiv Mux Select

  localparam mdm_x = 1'bx; // Don't Care
  localparam mdm_l = 1'd0; // Take lower half of 64-bit result, mul/div/divu
  localparam mdm_u = 1'd1; // Take upper half of 64-bit result, rem/remu

  // Execute Mux Select

  localparam em_x   = 1'bx; // Don't Care
  localparam em_alu = 1'd0; // Use ALU output
  localparam em_md  = 1'd1; // Use muldiv output

  // Memory Request Type

  localparam nr = 2'b0; // No request
  localparam ld = 2'd1; // Load
  localparam st = 2'd2; // Store

  // Subword Memop Length

  localparam ml_x  = 2'bx;
  localparam ml_w  = 2'd0;
  localparam ml_b  = 2'd1;
  localparam ml_h  = 2'd2;

  // Memory Response Mux Select

  localparam dmm_x  = 3'bx;
  localparam dmm_w  = 3'd0;
  localparam dmm_b  = 3'd1;
  localparam dmm_bu = 3'd2;
  localparam dmm_h  = 3'd3;
  localparam dmm_hu = 3'd4;

  // Writeback Mux 1

  localparam wm_x   = 1'bx; // Don't care
  localparam wm_alu = 1'd0; // Use ALU output
  localparam wm_mem = 1'd1; // Use data memory response

//--------------------------------------------------------------------
// PC Logic Stage
//--------------------------------------------------------------------

  // PC mux

  // DISCLAIMER: Changed the pc_plus4_Phl from wire to reg
  //wire [31:0] pc_plus4_Phl;
  reg [31:0] pc_plus4_Phl;

  reg brj_taken_Phl;
  //wire [31:0] branch_targ_Phl;
  reg [31:0] branch_targ_Phl;
  reg [31:0] jump_targ_Phl;
  reg [31:0] jumpreg_targ_Phl;
  wire [31:0] pc_mux_out_Phl;

  wire [31:0] reset_vector = 32'h00080000;

  wire [31:0] reset_mux_out;

  // ADDED BY ME
  // Allows pc to go back to reset_vector, using the "reset" control signal provided as input

  assign reset_mux_out = (reset) ? reset_vector : (pc_mux_out_Phl); //c hardcoded to not have branchs

  // Note: This is quite different from what we have before
  // This way, the pc can be updated without waiting for the branch signals.
  // Pc+4 should come immediately from the same satge insted of waiting for execute stage
  // (I pretended to do it like that on the first try)

  wire [31:0] pc_plus4_value;

  // Just add 4 to whathever comes from reset_mux
  // When the program starts, help us move forward
  // When moving to a new addres, continue addign 4 there and "go line by line"

  assign pc_plus4_value = reset_mux_out + 3'd4; // TODO (Done) 
  

  always @ (posedge clk) begin
    if( reset ) begin
      pc_plus4_Phl <= pc_plus4_value;
      //brj_taken_Xhl <= 1'b0;  //temp removed
      //brj_taken_Phl <= 1'b0;  //temp removed

    end
    else if( !stall_Fhl ) begin                       // WARNING: Check this later --- WARNING 2: Changed it to work onlt if F not stalled
      //pc_plus4_Phl <= pc_plus4_value;
      branch_targ_Phl <= branch_targ_Xhl;

      //brj_taken_Phl <= (brj_taken_Xhl)? 1'b0 : 1'b1; //temp removed 
    end
    else begin                       // WARNING: Check this later --- WARNING 2: Changed it to work onlt if F not stalled
      pc_plus4_Phl <= pc_plus4_value;
      branch_targ_Phl <= branch_targ_Xhl;

      //rj_taken_Phl <= (brj_taken_Xhl)? 1'b0 : 1'b1; //temp removed
    end
  end

  // Pull mux inputs from later stages
  
  

  // Older implementation. I saw that the pc had a truckload of problems so the next implementation wil be a lot different, I further separate the functinality 
  // of the PC_mux into two
  /*
  assign pc_mux_out_Phl =     // TODO (Done) Branching is a litlle more complex, we may do a "misprediction"
    ( _Phl == pm_p) ? pc_plus4_Phl :
    (pc_mux_sel_Phl == pm_b) ? branch_targ_Phl :            // Take the branch if, in EX stage, we found that it should be taken
    (pc_mux_sel_Phl == pm_j) ? jump_targ_Phl :
    (pc_mux_sel_Phl == pm_r) ? jumpreg_targ_Phl :
    pc_plus4_Phl;  // jump to reset vector, prevents fetching garbage
  */

  // Separate pc+4 from other branch addresses, calculated in the execute stage. If no branchs are taken, we just go line by line over the instrucions

  assign pc_mux_out_Phl = pc_plus4_Fhl;// TODO (Done) //c hardcoided this to try to make pc+4 work DAMN
    //(!brj_taken_Xhl)     ? pc_plus4_Phl   :  
    //(brj_taken_Xhl) ? jump_targ_valid_Phl     :
    //pc_plus4_Phl;

  // This part is intended to act as a way to check wether the chosen addresss is valid in this cycle
  // The signal has to go to CoreCtrl and be used to determine if imemreq_val

  wire pc_mux_out_valid_Phl =
   (brj_mux_sel_Xhl == pm_p) ? 1'b1                   :
   (brj_mux_sel_Xhl == pm_b) ? branch_targ_valid_Phl    :
   (brj_mux_sel_Xhl == pm_j) ? jump_targ_valid_Phl      :
   jumpreg_targ_valid_Phl;

  // Send out imem request early
  // Part of modification for Ctrl/Dpath connection (PCFix)
  assign imemreq_msg_addr = ( reset ) ? reset_vector : pc_Fhl;

//----------------------------------------------------------------------
// F <- P
//----------------------------------------------------------------------

  reg  [31:0] pc_Fhl;

  always @ (posedge clk) begin
    if( reset ) begin
      pc_Fhl <= reset_vector;
    end
    else if( !stall_Fhl ) begin
      pc_Fhl <= reset_mux_out; //c changed this 
    end
  end

//--------------------------------------------------------------------
// Fetch Stage
//--------------------------------------------------------------------

  // PC incrementer

  wire [31:0] pc_plus4_Fhl;

  assign pc_plus4_Fhl = pc_Fhl + 32'd4;

//----------------------------------------------------------------------
// D <- F
//----------------------------------------------------------------------
  
  // DISCLAIMER: Modifications to implement forwarding unit (Take values for Eecution from other stages)
  //reg [4:0] rs1_Xhl, rs2_Xhl;

  reg [31:0] pc_Dhl;
  reg [31:0] pc_plus4_Dhl;

  always @ (posedge clk) begin
    if( !stall_Dhl ) begin
      pc_Dhl       <= pc_Fhl;
      pc_plus4_Dhl <= pc_plus4_Fhl;
    end
  end

//--------------------------------------------------------------------
// Decode Stage (Register Read)
//------------------------------------- -------------------------------
  
  // Parse instruction fields

  wire   [4:0] inst_rs1_Dhl;
  wire   [4:0] inst_rs2_Dhl;
  wire   [4:0] inst_rd_Dhl;
  wire   [4:0] inst_shamt_Dhl;      // Gets extended, then goes to op1_mux
  wire  [31:0] imm_i_Dhl;           // Used with Jr_targ, also gets extended, then goes to op1_mux
  wire  [31:0] imm_u_Dhl;           // Gets extended, then goes to op1_mux
  wire  [31:0] imm_uj_Dhl;          // Used with J_targ
  wire  [31:0] imm_s_Dhl;           // Gets extended, then goes to op1_mux
  wire  [31:0] imm_sb_Dhl;          // Used with br_targ

  // Register file

  wire [ 4:0] rf_raddr0_Dhl = inst_rs1_Dhl;
  wire [31:0] rf_rdata0_Dhl;
  wire [ 4:0] rf_raddr1_Dhl = inst_rs2_Dhl;
  wire [31:0] rf_rdata1_Dhl;

  // Constant operand mux inputs

  wire [31:0] const0    = 32'd0;


  // Operand 0 mux

  //  we  pipelinerize it XDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
  wire [31:0] rs1_val_Dhl = rf_rdata0_Dhl;
  wire [31:0] rs2_val_Dhl = rf_rdata1_Dhl;




  wire [31:0] op0_mux_out_Dhl =  // TODO (Done)  similar to single-cycle but check for "most recent value"
    (op0_mux_sel_Dhl == am_rdat)  ? rs1_val_Dhl:     // Use correct value, avoid using old regsiter value
    (op0_mux_sel_Dhl == am_pc)    ? pc_Dhl :            // PC
    (op0_mux_sel_Dhl == am_pc4)   ? pc_plus4_Dhl :      // PC + 4
    (op0_mux_sel_Dhl == am_0)     ? 32'd0 :         // 0 Constant
    reset_vector;  // jump to reset vector, prevents fetching garbage 
  // Operand 1 mux

  wire [31:0] op1_mux_out_Dhl =  // TODO (Done) pretty much the same as op0
    (op1_mux_sel_Dhl == bm_rdat)    ? rs2_val_Xhl:      // Use correct value, avoid using old regsiter value
    (op1_mux_sel_Dhl == bm_shamt)   ? imm_shamt_Xhl :     // Extended 5 bits imm
    (op1_mux_sel_Dhl == bm_imm_u)   ? imm_u_Xhl :
    (op1_mux_sel_Dhl == bm_imm_sb)  ? imm_sb_Xhl :
    (op1_mux_sel_Dhl == bm_imm_i)   ? imm_i_Xhl :
    (op1_mux_sel_Dhl == bm_imm_s)   ? imm_s_Xhl :
    (op1_mux_sel_Dhl == bm_0  )     ? 32'd0 :
    reset_vector;  // jump to reset vector, prevents fetching garbage 

  // ADDED TO COMPLETE PC LOGIC
  // Decode stage jump target that must reach P
  wire [31:0] jump_targ_Dhl     = pc_Dhl + imm_uj_Dhl; 
  //wire        is_jal_D        = (opcode_D == OP_JAL); //c


  // wdata with bypassing

  wire [31:0] wdata_Dhl = rf_rdata1_Dhl;

  // DISCLAIMER: declaration of missing reg branch_targ_Dhl
  //reg [31:0] branch_targ_Dhl;

  always @(posedge clk) begin
    if ( reset ) begin
      //branch_targ_Dhl    <= 32'b0;
    end
    else if ( !stall_Dhl ) begin
      // Compute the static target PC = PC+D + imm_sb
      //branch_targ_Dhl  <= pc_Dhl + sb_shamt; // Add the PC + exended shift amount
    end
  end

//----------------------------------------------------------------------
// X <- D
//----------------------------------------------------------------------

  reg [31:0] pc_Xhl;
  //reg [31:0] branch_targ_Xhl;
  reg [31:0] op0_mux_out_Xhl;
  reg [31:0] op1_mux_out_Xhl;
  reg [31:0] wdata_Xhl;

  // ADDED TO COMPLETE PC LOGIC
  reg [31:0] imm_i_Xhl, imm_sb_Xhl, imm_uj_Xhl, imm_s_Xhl, imm_u_Xhl, imm_shamt_Xhl;      // branch-/jalr-imms seen by X

  reg[31:0] rs1_val_Xhl, rs2_val_Xhl;

  // ADDED TO COMPLETE PC LOGIC
  always @ (posedge clk) begin
    if(reset) begin
      imm_i_Xhl <= 32'd0;
      imm_sb_Xhl <= 32'd0;
      imm_uj_Xhl <= 32'd0;
      imm_s_Xhl <= 32'd0;
      imm_u_Xhl <= 32'd0;
      imm_shamt_Xhl <= 32'd0;
    end
    else if( !stall_Xhl ) begin
      pc_Xhl          <= pc_Dhl;
      //branch_targ_Xhl <= branch_targ_Dhl;
      op0_mux_out_Xhl <= op0_mux_out_Dhl;
      op1_mux_out_Xhl <= op1_mux_out_Dhl;
      wdata_Xhl       <= wdata_Dhl;

      // ADDED TO COMPLETE PC LOGIC (I discovered i don't use some, but theyre still there)
      imm_i_Xhl <= imm_i_Dhl;               // from decoder output-bus
      imm_sb_Xhl <= imm_sb_Dhl;
      imm_uj_Xhl <= imm_uj_Dhl;

      imm_i_Xhl <= imm_i_Dhl;
      imm_s_Xhl <= imm_s_Dhl;
      imm_u_Xhl <= imm_u_Dhl;
      imm_shamt_Xhl <= {27'b0, inst_shamt_Dhl};
      imm_sb_Xhl <= imm_sb_Dhl;

      rs1_val_Xhl <= rs1_val_Dhl;
      rs2_val_Xhl <= rs2_val_Dhl;
    end
  end

//----------------------------------------------------------------------
// Execute Stage
//----------------------------------------------------------------------

  // ALU

  wire [31:0] alu_out_Xhl;

  // Branch condition logic

  // Conditions are the same, just comparisons. These signals get sent to other units
  // Get the value from the mux's and compare them
  assign branch_cond_eq_Xhl    = (op0_mux_out_Xhl == op1_mux_out_Xhl); // TODO (Done)
  assign branch_cond_ne_Xhl    = (op0_mux_out_Xhl != op1_mux_out_Xhl); // TODO (Done)
  assign branch_cond_lt_Xhl    = ($signed(op0_mux_out_Xhl) < $signed(op1_mux_out_Xhl)); // TODO (Done)
  assign branch_cond_ltu_Xhl   = (op0_mux_out_Xhl < op1_mux_out_Xhl); // TODO (Done)
  assign branch_cond_ge_Xhl    = ($signed(op0_mux_out_Xhl) >= $signed(op1_mux_out_Xhl)); // TODO  (Done)
  assign branch_cond_geu_Xhl   = (op0_mux_out_Xhl >= op1_mux_out_Xhl); // TODO (Done)

  // Send out memory request during X, response returns in M

  // The following two may require adjustment based on possible hazards
  assign dmemreq_msg_addr = alu_out_Xhl; // TODOO (done) EA from ALU
  assign dmemreq_msg_data = wdata_Xhl; // TODOO (done) store data

  // Muldiv Unit

  wire [63:0] muldivresp_msg_result_Xhl;

  // Muldiv Result Mux

  // The result changes depending on the instruction
  wire [31:0] muldiv_mux_out_Xhl =  // TODO (Done) same as bf
    (muldiv_mux_sel_Xhl == mdm_l)     ?  muldivresp_msg_result_Xhl[31:0] :    // Take lower half of 64-bit result, mul/div/divu
    (muldiv_mux_sel_Xhl == mdm_u)     ?  muldivresp_msg_result_Xhl[63:32] :    // Take upper half of 64-bit result, rem/remu
    32'd0;
  // Execute Result Mux

  wire [31:0] execute_mux_out_Xhl =  // TODO (Done) same as bf
    (execute_mux_sel_Xhl ==  em_alu)  ?   alu_out_Xhl:          // Use ALU output
    (execute_mux_sel_Xhl ==  em_md)   ?   muldiv_mux_out_Xhl:   // Use muldiv output
    32'd0;


// ADDED TO COMPLETE PC LOGIC
// Execute-stage results that must reach P
wire [31:0] branch_targ_Xhl   = pc_Xhl + imm_i_Xhl;
wire        branch_take_Xhl   = brj_taken_Xhl; //Given as input from ctrl

wire [31:0] jumpreg_targ_Xhl  = rs1_val_Xhl + imm_i_Xhl; 

wire [31:0] brj_mux_out =
  (brj_mux_sel_Xhl ==  pm_x)    ?   32'hDEADBEEF:          // don't care xd
  (brj_mux_sel_Xhl ==  pm_p)    ?   muldiv_mux_out_Xhl:   // Use muldiv output
  (brj_mux_sel_Xhl ==  pm_b)    ?   alu_out_Xhl:          // Use ALU output
  (brj_mux_sel_Xhl ==  pm_j)    ?   muldiv_mux_out_Xhl:   // Use muldiv output
  (brj_mux_sel_Xhl ==  pm_r)    ?   muldiv_mux_out_Xhl:   // Use muldiv output
  32'd0;
//c (dunno what to left here exactly)
// Pipeline into P  (gated by the *P* stage stall)
//reg [31:0] branch_targ_Phl, jumpreg_targ_Phl, jump_targ_Phl; ALREADY DECLARED
reg        branch_targ_valid_Phl, jumpreg_targ_valid_Phl, jump_targ_valid_Phl;

always @(posedge clk) begin
  if (reset) begin
    branch_targ_valid_Phl  <= 1'b0;
    jump_targ_valid_Phl    <= 1'b0;
    jumpreg_targ_valid_Phl <= 1'b0;
  end else  begin                             // WARNING: check later //c
    branch_targ_Phl        <= branch_targ_Xhl;
    branch_targ_valid_Phl  <= branch_take_Xhl;

    jumpreg_targ_Phl       <= jumpreg_targ_Xhl;
    jumpreg_targ_valid_Phl <= 1'b1;                 // jalr always produces a target

    jump_targ_Phl          <= jump_targ_Dhl;
    //jump_targ_valid_Phl    <= is_jal_D;
  end
end


//----------------------------------------------------------------------
// M <- X
//----------------------------------------------------------------------

  reg  [31:0] pc_Mhl;
  reg  [31:0] execute_mux_out_Mhl;
  reg  [31:0] wdata_Mhl;

  always @ (posedge clk) begin
    if( !stall_Mhl ) begin
      pc_Mhl              <= pc_Xhl;
      execute_mux_out_Mhl <= execute_mux_out_Xhl;
      wdata_Mhl           <= wdata_Xhl;
    end
  end

//----------------------------------------------------------------------
// Memory Stage
//----------------------------------------------------------------------

  // Data memory subword adjustment mux

  // Adjust depending on the instruction
  wire [31:0] dmemresp_mux_out_Mhl =  // TODO (done)
    (dmemresp_mux_sel_Mhl == dmm_w)  ? dmemresp_msg_data :                                      // Full 32-bit word
    (dmemresp_mux_sel_Mhl == dmm_b)  ? {{24{dmemresp_msg_data[7]}}, dmemresp_msg_data[7:0]} :   // Byte (sign-extended)
    (dmemresp_mux_sel_Mhl == dmm_bu) ? {24'b0, dmemresp_msg_data[7:0]} :                        // Byte (zero-extended)
    (dmemresp_mux_sel_Mhl == dmm_h)  ? {{16{dmemresp_msg_data[15]}}, dmemresp_msg_data[15:0]} : // Half-word (sign-extended)
    (dmemresp_mux_sel_Mhl == dmm_hu) ? {16'b0, dmemresp_msg_data[15:0]} :                       // Half-word (zero-extended)
    32'b0;  // Default case

//----------------------------------------------------------------------
// Queue for data memory response
//----------------------------------------------------------------------

  reg [31:0] dmemresp_queue_reg_Mhl;

  always @ ( posedge clk ) begin
    if ( dmemresp_queue_en_Mhl ) begin
      dmemresp_queue_reg_Mhl <= dmemresp_mux_out_Mhl;
    end
  end

//----------------------------------------------------------------------
// Data memory queue mux
//----------------------------------------------------------------------

  wire [31:0] dmemresp_queue_mux_out_Mhl
    = ( !dmemresp_queue_val_Mhl ) ? dmemresp_mux_out_Mhl
    : ( dmemresp_queue_val_Mhl )  ? dmemresp_queue_reg_Mhl
    :                               32'bx;

//----------------------------------------------------------------------
// Writeback mux
//----------------------------------------------------------------------

  wire [31:0] wb_mux_out_Mhl =  // TODO (done)
    (wb_mux_sel_Mhl == wm_alu)  ?   execute_mux_out_Mhl :          // Use ALU output
    (wb_mux_sel_Mhl == wm_mem)  ?   dmemresp_mux_out_Mhl:  // Use data memory response
    32'b0;  // Default case

//----------------------------------------------------------------------
// W <- M
//----------------------------------------------------------------------

  reg  [31:0] pc_Whl;
  reg  [31:0] wb_mux_out_Whl;

  always @ (posedge clk) begin
    if( !stall_Whl ) begin
      pc_Whl                 <= pc_Mhl;
      wb_mux_out_Whl         <= wb_mux_out_Mhl;
    end
  end

//----------------------------------------------------------------------
// Writeback Stage
//----------------------------------------------------------------------

  // CSR write data

  assign proc2csr_data_Whl = wb_mux_out_Whl;

//----------------------------------------------------------------------
// Debug registers for instruction disassembly
//----------------------------------------------------------------------

  reg [31:0] pc_debug;

  always @ ( posedge clk ) begin
    pc_debug <= pc_Whl;
  end
  
//----------------------------------------------------------------------
// Submodules
//----------------------------------------------------------------------
  
  // Address Generation

  riscv_InstMsgFromBits inst_msg_from_bits
  (
    .msg      (inst_Dhl),
    .opcode   (),
    .rs1      (inst_rs1_Dhl),
    .rs2      (inst_rs2_Dhl),
    .rd       (inst_rd_Dhl),
    .funct3   (),
    .funct7   (),
    .shamt    (inst_shamt_Dhl),
    .imm_i    (imm_i_Dhl),
    .imm_s    (imm_s_Dhl),
    .imm_sb   (imm_sb_Dhl),
    .imm_u    (imm_u_Dhl),
    .imm_uj   (imm_uj_Dhl)
  );

  // Register File

  riscv_CoreDpathRegfile rfile
  (
    .clk     (clk),
    .raddr0  (rf_raddr0_Dhl),
    .rdata0  (rf_rdata0_Dhl),
    .raddr1  (rf_raddr1_Dhl),
    .rdata1  (rf_rdata1_Dhl),
    .wen_p   (rf_wen_Whl),
    .waddr_p (rf_waddr_Whl),
    .wdata_p (wb_mux_out_Whl)
  );

  // ALU

  riscv_CoreDpathAlu alu
  (
    .in0  (op0_mux_out_Xhl),
    .in1  (op1_mux_out_Xhl),
    .fn   (alu_fn_Xhl),
    .out  (alu_out_Xhl)
  );

  // Multiplier/Divider

  riscv_CoreDpathMulDiv muldiv 
  (
    .clk                   (clk),
    .reset                 (reset),
    .muldivreq_msg_fn      (muldivreq_msg_fn_Xhl),
    .muldivreq_msg_a       (op0_mux_out_Xhl),
    .muldivreq_msg_b       (op1_mux_out_Xhl),
    .muldivreq_val         (muldivreq_val),
    .muldivreq_rdy         (muldivreq_rdy),
    .muldivresp_msg_result (muldivresp_msg_result_Xhl),
    .muldivresp_val        (muldivresp_val),
    .muldivresp_rdy        (muldivresp_rdy)
  );

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
