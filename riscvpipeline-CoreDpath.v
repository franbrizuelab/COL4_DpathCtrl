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

  input   [1:0] pc_mux_sel_Phl,
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

  // DISCLAIMER: Modifications to implement forwarding unit (Take values for Eecution from other stages)
  //input        rf_wen_Mhl,
  //input [4:0]  rf_waddr_Mhl,
  //input [31:0] execute_mux_out_Mhl,
  

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

  // PC Mux Select

  localparam pm_x   = 2'bx;  // Don't care
  localparam pm_p   = 2'd0;  // Use pc+4
  localparam pm_b   = 2'd1;  // Use branch address
  localparam pm_j   = 2'd2;  // Use jump address
  localparam pm_r   = 2'd3;  // Use jump register

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

  wire [31:0] pc_plus4_Phl;
  wire [31:0] branch_targ_Phl;
  wire [31:0] jump_targ_Phl;
  wire [31:0] jumpreg_targ_Phl;
  wire [31:0] pc_mux_out_Phl;

  wire [31:0] reset_vector = 32'h00080000;

  //Note: pc-plus4_Phl is unsassigned. We need to find the signal to assign a value to it

  assign pc_plus4_Phl =
    (pc_Fhl == 32'bx) ?  reset_vector:
    pc_Fhl + 32'd4;
  
  // Pull mux inputs from later stages

  assign pc_mux_out_Phl =     // TODO (Done) SHould be the same as last time
    (pc_mux_sel_Phl == pm_p) ? pc_plus4_Phl :
    (pc_mux_sel_Phl == pm_b) ? branch_targ_Phl :
    (pc_mux_sel_Phl == pm_j) ? jump_targ_Phl :
    (pc_mux_sel_Phl == pm_r) ? jumpreg_targ_Phl :
    reset_vector;  // jump to reset vector, prevents fetching garbage

  // Send out imem request early

  assign imemreq_msg_addr
    = ( reset ) ? reset_vector
    :             pc_mux_out_Phl;

//----------------------------------------------------------------------
// F <- P
//----------------------------------------------------------------------

  reg  [31:0] pc_Fhl;

  always @ (posedge clk) begin
    if( reset ) begin
      pc_Fhl <= reset_vector;
    end
    else if( !stall_Fhl ) begin
      pc_Fhl <= pc_mux_out_Phl;
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
  wire   [4:0] inst_shamt_Dhl;
  wire  [31:0] imm_i_Dhl;
  wire  [31:0] imm_u_Dhl;
  wire  [31:0] imm_uj_Dhl;
  wire  [31:0] imm_s_Dhl;
  wire  [31:0] imm_sb_Dhl;

  // Register file

  wire [ 4:0] rf_raddr0_Dhl = inst_rs1_Dhl;
  wire [31:0] rf_rdata0_Dhl;
  wire [ 4:0] rf_raddr1_Dhl = inst_rs2_Dhl;
  wire [31:0] rf_rdata1_Dhl;

  // Shift amount immediate

  wire [31:0] shamt_Dhl = {27'b0, inst_shamt_Dhl}; // TODO (Done) extend the original 5 bits

  // Constant operand mux inputs

  wire [31:0] const0    = 32'd0;


  // Operand 0 mux

  // For a first try, directly grab the output from rf_dataX-Dhl
  wire [31:0] rs1_val_D = rf_rdata0_Dhl;
  wire [31:0] rs2_val_D = rf_rdata1_Dhl;

  wire [31:0] op0_mux_out_Dhl =  // TODO (Done)  similar to single-cycle but check for "most recent value"
    (op0_mux_sel_Dhl == am_rdat)  ? rs1_val_D:     // Use correct value, avoid using old regsiter value
    (op0_mux_sel_Dhl == am_pc)    ? pc_Dhl :            // PC
    (op0_mux_sel_Dhl == am_pc4)   ? pc_plus4_Dhl :      // PC + 4
    (op0_mux_sel_Dhl == am_0)     ? 32'd0 :         // 0 Constant
    reset_vector;  // jump to reset vector, prevents fetching garbage 
  // Operand 1 mux

  wire [31:0] op1_mux_out_Dhl =  // TODO (Done) pretty much the same as op0
    (op1_mux_sel_Dhl == bm_rdat)    ? rs2_val_D:      // Use correct value, avoid using old regsiter value
    (op1_mux_sel_Dhl == bm_shamt)   ? shamt_Dhl :     // Extended 5 bits imm
    (op1_mux_sel_Dhl == bm_imm_u)   ? imm_u_Dhl :
    (op1_mux_sel_Dhl == bm_imm_sb)  ? imm_sb_Dhl :
    (op1_mux_sel_Dhl == bm_imm_i)   ? imm_i_Dhl :
    (op1_mux_sel_Dhl == bm_imm_s)   ? imm_s_Dhl :
    (op1_mux_sel_Dhl == bm_0  )       ? 32'd0 :
    reset_vector;  // jump to reset vector, prevents fetching garbage 
    
  // wdata with bypassing

  wire [31:0] wdata_Dhl = rf_rdata1_Dhl;

//----------------------------------------------------------------------
// X <- D
//----------------------------------------------------------------------

  reg [31:0] pc_Xhl;
  reg [31:0] branch_targ_Xhl;
  reg [31:0] op0_mux_out_Xhl;
  reg [31:0] op1_mux_out_Xhl;
  reg [31:0] wdata_Xhl;

  always @ (posedge clk) begin
    if( !stall_Xhl ) begin
      pc_Xhl          <= pc_Dhl;
      //branch_targ_Xhl <= branch_targ_Dhl;
      op0_mux_out_Xhl <= op0_mux_out_Dhl;
      op1_mux_out_Xhl <= op1_mux_out_Dhl;
      wdata_Xhl       <= wdata_Dhl;

      // DISCLAIMER: Modifications to implement forwarding unit (Take values for Eecution from other stages)
      //rs1_Xhl <= inst_rs1_Dhl;    // from your InstMsg parser
      //rs2_Xhl <= inst_rs2_Dhl;
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
