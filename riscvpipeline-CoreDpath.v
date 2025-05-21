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

  // Control Signals (dpath->ctrl)

  output        branch_cond_eq_Xhl,
  output        branch_cond_ne_Xhl,
  output        branch_cond_lt_Xhl,
  output        branch_cond_ltu_Xhl,
  output        branch_cond_ge_Xhl,
  output        branch_cond_geu_Xhl,
  output [31:0] proc2csr_data_Whl
);

//--------------------------------------------------------------------
// PC Logic Stage
//--------------------------------------------------------------------

  // PC mux

  //wire [31:0] pc_plus4_Phl;
  wire [31:0] branch_targ_Phl;
  wire [31:0] jump_targ_Phl;
  wire [31:0] jumpreg_targ_Phl;
  wire [31:0] pc_mux_out_Phl;

  wire [31:0] reset_vector = 32'h00080000;

  // Pull mux inputs from later stages

  // pass target addresses forward
  assign branch_targ_Phl  = branch_targ_Xhl;
  assign jump_targ_Phl    = jump_targ_Xhl;
  assign jumpreg_targ_Phl = jumpreg_targ_Xhl;

  assign pc_mux_out_Phl = 
    (pc_mux_sel_Phl == 2'd0) ? pc_plus4_Fhl     :
    (pc_mux_sel_Phl == 2'd1) ? branch_targ_Phl  :
    (pc_mux_sel_Phl == 2'd2) ? jump_targ_Phl    :
    (pc_mux_sel_Phl == 2'd3) ? jumpreg_targ_Phl :
                               32'bx; // TODO - done

  // Send out imem request early

  assign imemreq_msg_addr
    = ( reset ) ? reset_vector
    :             pc_Fhl;

//----------------------------------------------------------------------
// F <- P
//----------------------------------------------------------------------

  reg  [31:0] pc_Fhl;
  reg init_stall;

  always @ (posedge clk) begin         
    if( reset ) begin
      init_stall <= 1'b1;
      pc_Fhl <= reset_vector;
    end
    else begin
      init_stall <= 1'b0;
      if (!stall_Fhl && !init_stall) begin
        pc_Fhl <= pc_mux_out_Phl;
      end
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
//--------------------------------------------------------------------

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
  // Zero-extend the 5-bit shift amount
  wire [31:0] shamt_Dhl = {27'd0, inst_shamt_Dhl}; // TODO - done

  // Constant operand mux inputs

  wire [31:0] const0    = 32'd0;

  // Replace the original mux with forwarding-aware muxes
  

  // Operand 0 mux

  wire [31:0] op0_mux_out_Dhl = 
    (op0_mux_sel_Dhl == 2'd0) ? rf_rdata0_Dhl :
    (op0_mux_sel_Dhl == 2'd1) ? pc_Dhl        :
    (op0_mux_sel_Dhl == 2'd2) ? pc_plus4_Dhl  :
    (op0_mux_sel_Dhl == 2'd3) ? const0        :
    32'bx; // TODO - DONE ?

  // Operand 1 mux

  wire [31:0] op1_mux_out_Dhl =  (op1_mux_sel_Dhl == 3'd0) ? rf_rdata1_Dhl :
    (op1_mux_sel_Dhl == 3'd1) ? shamt_Dhl     :
    (op1_mux_sel_Dhl == 3'd2) ? imm_u_Dhl     :
    (op1_mux_sel_Dhl == 3'd3) ? imm_sb_Dhl    :
    (op1_mux_sel_Dhl == 3'd4) ? imm_i_Dhl     :
    (op1_mux_sel_Dhl == 3'd5) ? imm_s_Dhl     :
    (op1_mux_sel_Dhl == 3'd6) ? const0        :
                                32'bx; // TODO - done

  // wdata with bypassing

  wire [31:0] wdata_Dhl = rf_rdata1_Dhl;

  // PC‐relative target (UJ‐type immediate)  
  wire [31:0] jump_targ_Dhl = pc_Dhl + imm_uj_Dhl;

  // Register‐indirect target (rs1 + I‐type immediate), low bit forced to zero
  wire [31:0] jumpreg_targ_Dhl = (rf_rdata0_Dhl + imm_i_Dhl) & 32'hFFFFFFFE; // added

  wire [31:0] branch_targ_Dhl = pc_Dhl + imm_sb_Dhl;



//----------------------------------------------------------------------
// X <- D
//----------------------------------------------------------------------

  reg [31:0] pc_Xhl;
  reg [31:0] branch_targ_Xhl;
  reg [31:0] op0_mux_out_Xhl;
  reg [31:0] op1_mux_out_Xhl;
  reg [31:0] wdata_Xhl;
  reg [31:0] jump_targ_Xhl; // added
  reg [31:0] jumpreg_targ_Xhl; // added

  always @ (posedge clk) begin
    if( !stall_Xhl ) begin
      pc_Xhl          <= pc_Dhl;
      branch_targ_Xhl <= branch_targ_Dhl;
      jump_targ_Xhl   <= jump_targ_Dhl; //added
      jumpreg_targ_Xhl<= jumpreg_targ_Dhl; //added
      op0_mux_out_Xhl <= op0_mux_out_Dhl;
      op1_mux_out_Xhl <= op1_mux_out_Dhl;
      wdata_Xhl       <= wdata_Dhl;
    end
  end

//----------------------------------------------------------------------
// Execute Stage
//----------------------------------------------------------------------

  // ALU

  wire [31:0] alu_out_Xhl;

  // Branch condition logic

  assign branch_cond_eq_Xhl    = (op0_mux_out_Xhl == op1_mux_out_Xhl); // TODO - done
  assign branch_cond_ne_Xhl    = (op0_mux_out_Xhl != op1_mux_out_Xhl); // TODO - done
  assign branch_cond_lt_Xhl    = ($signed(op0_mux_out_Xhl)    <   $signed(op1_mux_out_Xhl));    // TODO - done
  assign branch_cond_ltu_Xhl   = ($unsigned(op0_mux_out_Xhl)  <   $unsigned(op1_mux_out_Xhl));  // TODO - done
  assign branch_cond_ge_Xhl    = ($signed(op0_mux_out_Xhl)    >=  $signed(op1_mux_out_Xhl));    // TODO - done
  assign branch_cond_geu_Xhl   = ($unsigned(op0_mux_out_Xhl)  >=  $unsigned(op1_mux_out_Xhl));  // TODO - done

  // Send out memory request during X, response returns in M

  assign dmemreq_msg_addr = execute_mux_out_Xhl; // TODO - done
  assign dmemreq_msg_data = wdata_Xhl; // TODO - done

  // Muldiv Unit

  wire [63:0] muldivresp_msg_result_Xhl;

  // Muldiv Result Mux

  wire [31:0] muldiv_mux_out_Xhl = 
    (muldiv_mux_sel_Xhl == 1'b1) ? muldivresp_msg_result_Xhl[31:0]
                                 : alu_out_Xhl; // TODO - done

  // Execute Result Mux

  wire [31:0] execute_mux_out_Xhl = 
    (execute_mux_sel_Xhl == 1'b1) ? muldiv_mux_out_Xhl
                                  : alu_out_Xhl; // TODO - done

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

  wire [31:0] dmemresp_mux_out_Mhl = 
    (dmemresp_mux_sel_Mhl == 3'd0) ? dmemresp_msg_data :
    (dmemresp_mux_sel_Mhl == 3'd1) ? {{24{dmemresp_msg_data[7]}},  dmemresp_msg_data[7:0]} :
    (dmemresp_mux_sel_Mhl == 3'd2) ? {24'd0, dmemresp_msg_data[7:0]} :
    (dmemresp_mux_sel_Mhl == 3'd3) ? {{16{dmemresp_msg_data[15]}}, dmemresp_msg_data[15:0]} :
    (dmemresp_mux_sel_Mhl == 3'd4) ? {16'd0, dmemresp_msg_data[15:0]} :
                                      32'bx;


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

  wire [31:0] wb_mux_out_Mhl = (wb_mux_sel_Mhl == 1'b0) ? execute_mux_out_Mhl : dmemresp_queue_mux_out_Mhl; // TODO - done

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