//========================================================================
// Mul/Div Unit 
//========================================================================
 
`ifndef RISCV_PIPE_MULDIV_ITERATIVE_V
`define RISCV_PIPE_MULDIV_ITERATIVE_V
  
module riscv_CoreDpathMulDiv
(
  input         clk,
  input         reset,
 
  input   [2:0] muldivreq_msg_fn,
  input  [31:0] muldivreq_msg_a,
  input  [31:0] muldivreq_msg_b,
  input         muldivreq_val,
  output        muldivreq_rdy,
 
  output [63:0] muldivresp_msg_result,
  output        muldivresp_val,
  input         muldivresp_rdy
);
 
  //----------------------------------------------------------------------
  // FSM
  //---------------------------------------------------------------------- 
  reg  [1:0] state;
  localparam IDLE = 2'd0;
  localparam C1   = 2'd1;
  localparam C2   = 2'd2;
  localparam C3   = 2'd3;
 
  always @(posedge clk) begin
    if (reset) begin
      state <= IDLE;
    end else begin
      case (state)
        IDLE: if (muldivreq_val) state <= C1;
        C1:   state <= C2;
        C2:   state <= C3;
        C3:   if (muldivresp_rdy) state <= IDLE;
      endcase
    end
  end

  reg [2:0]  fn_reg;
  reg [31:0] a_reg;
  reg [31:0] b_reg;
 
  always @(posedge clk) begin
    if (reset) begin
      fn_reg <= 3'd0;
      a_reg  <= 32'd0;
      b_reg  <= 32'd0;
    end else if (state == IDLE && muldivreq_val) begin
      fn_reg <= muldivreq_msg_fn;
      a_reg  <= muldivreq_msg_a;
      b_reg  <= muldivreq_msg_b;
    end
  end
 
  //----------------------------------------------------------------------
  // Functional Computation
  //----------------------------------------------------------------------
 
  wire sign = ( a_reg[31] ^ b_reg[31] );
 
  wire [31:0] a_unsign   = ( a_reg[31] == 1'b1 ) ? ( ~a_reg + 1'b1 ) : a_reg;
  wire [31:0] b_unsign   = ( b_reg[31] == 1'b1 ) ? ( ~b_reg + 1'b1 ) : b_reg;
 
  wire [31:0] quotientu  = a_reg / b_reg;
  wire [31:0] remainderu = a_reg % b_reg;
 
  wire [63:0] product_raw   = a_unsign * b_unsign;
  wire [31:0] quotient_raw  = a_unsign / b_unsign;
  wire [31:0] remainder_raw = a_unsign % b_unsign;
 
  wire [63:0] product   = ( sign ) ? ( ~product_raw + 1'b1 ) : product_raw;
  wire [31:0] quotient  = ( sign ) ? ( ~quotient_raw + 1'b1 ) : quotient_raw;
  wire [31:0] remainder = ( a_reg[31] ) ? ( ~remainder_raw + 1'b1 ) : remainder_raw;
 
  wire [63:0] result
    = ( fn_reg == 3'd0 ) ? product
    : ( fn_reg == 3'd1 ) ? { remainder, quotient }
    : ( fn_reg == 3'd2 ) ? { remainderu, quotientu }
    : ( fn_reg == 3'd3 ) ? { remainder, quotient }
    : ( fn_reg == 3'd4 ) ? { remainderu, quotientu }
    :                      64'bx;
 
  //----------------------------------------------------------------------
  // Output Assignment
  //----------------------------------------------------------------------
 
  assign muldivreq_rdy = (state == IDLE);
  assign muldivresp_val = (state == C3);
  assign muldivresp_msg_result = result;
 
endmodule
 
`endif