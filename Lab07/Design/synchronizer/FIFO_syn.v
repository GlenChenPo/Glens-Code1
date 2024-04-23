module FIFO_syn #(parameter WIDTH=8, parameter WORDS=64) (
    wclk,
    rclk,
    rst_n,
    winc,
    wdata,
    wfull,
    rinc,
    rdata,
    rempty,

    flag_fifo_to_clk2,
    flag_clk2_to_fifo,

    flag_fifo_to_clk1,
	flag_clk1_to_fifo
);

input wclk, rclk;
input rst_n;
input winc;
input [WIDTH-1:0] wdata;
output reg wfull;
input rinc;
output reg [WIDTH-1:0] rdata;
output reg rempty;

// You can change the input / output of the custom flag ports
output  flag_fifo_to_clk2;
input flag_clk2_to_fifo;

output flag_fifo_to_clk1;
input flag_clk1_to_fifo;

wire [WIDTH-1:0] rdata_q;

// Remember: 
//   wptr and rptr should be gray coded
//   Don't modify the signal name
reg [$clog2(WORDS):0] wptr;
reg [$clog2(WORDS):0] rptr;

// rdata
//  Add one more register stage to rdata
always @(posedge rclk, negedge rst_n) begin
    if (!rst_n) begin
        rdata <= 0;
    end
    else begin
		//if (rinc & !rempty) begin
			rdata <= rdata_q;
		end
    end

//==========================================
//             reg
//==========================================
wire write_enable;
wire wen_a;
reg winc_q;
wire wfull_q;

wire rempty_q;
wire read_enable;


wire [6:0]w_addr_q;
wire [6:0]r_addr_grey;

reg  [6:0]w_addr;
reg  [6:0]r_addr;
// pointer
wire [$clog2(WORDS):0] rptr_q;
wire [$clog2(WORDS):0] wptr_q;
reg  [$clog2(WORDS):0] wq2_rptr;
reg  [$clog2(WORDS):0] rq2_wptr;

// counter
reg [8:0]cnt_write;
reg [8:0]cnt_read;
//----- ip --------------------------------
NDFF_BUS_syn #(.WIDTH(7)) sync_w2r (.D(wptr), .Q(rq2_wptr), .clk(rclk), .rst_n(rst_n));
NDFF_BUS_syn #(.WIDTH(7)) sync_r2w (.D(rptr), .Q(wq2_rptr), .clk(wclk), .rst_n(rst_n));


//=============================================
//            WRITE
//=============================================
assign write_enable = (winc & !wfull);
assign wen_a = !write_enable;
assign w_addr_q = w_addr + write_enable;
assign wptr_q  = (w_addr_q >> 1) ^ w_addr_q;
assign wfull_q = (wptr_q == {~wq2_rptr[$clog2(WORDS):$clog2(WORDS)-1], wq2_rptr[$clog2(WORDS)-2:0]});

always @(posedge wclk or negedge rst_n) begin
    if(!rst_n)     cnt_write <= 0;
    else           cnt_write <= cnt_write + write_enable;
end
always @(posedge wclk or negedge rst_n) begin
    if(!rst_n)      wfull <= 0;
    else            wfull <= wfull_q;
end
always @(posedge wclk or negedge rst_n) begin
    if(!rst_n)      wptr <= 0;
    else            wptr <= wptr_q;
end
always @(posedge wclk or negedge rst_n) begin
    if(!rst_n)     w_addr <= 0;
    else           w_addr <= w_addr_q;
end
always @(posedge wclk or negedge rst_n) begin
    if(!rst_n)      winc_q <= 0;
    else            winc_q <= winc;
end

assign flag_fifo_to_clk1 = r_addr[0];
//===================================================
//              READ
//===================================================
assign read_enable = (rinc & ~rempty);
assign r_addr_grey  = r_addr + read_enable;
assign rptr_q   = (r_addr_grey >> 1) ^ r_addr_grey;
assign rempty_q = (rptr_q == rq2_wptr);

  always @(posedge rclk or negedge rst_n) begin
      if(!rst_n)    cnt_read <= 0;
      else          cnt_read <= cnt_read + read_enable;
  end
  always @(posedge rclk or negedge rst_n) begin
      if(!rst_n)  rptr  <= 0;
      else        rptr  <= rptr_q;
  end
  always @(posedge rclk or negedge rst_n) begin
      if(!rst_n)  r_addr <= 0;
      else        r_addr <= r_addr + read_enable;
  end

  always @(posedge rclk or negedge rst_n) begin
      if(!rst_n)      rempty <= 1;
      else            rempty <= (rptr_q == rq2_wptr);
  end


DUAL_64X8X1BM1 u_dual_sram (
    .CKA(wclk),
    .CKB(rclk),
    .WEAN(wen_a), 
    .WEBN(1'b1),  // READ
    .CSA(1'b1),
    .CSB(1'b1),
    .OEA(1'b1),
    .OEB(1'b1),
    .A0(w_addr[0]),
    .A1(w_addr[1]),
    .A2(w_addr[2]),
    .A3(w_addr[3]),
    .A4(w_addr[4]),
    .A5(w_addr[5]),
    .B0(r_addr[0]),
    .B1(r_addr[1]),
    .B2(r_addr[2]),
    .B3(r_addr[3]),
    .B4(r_addr[4]),
    .B5(r_addr[5]),
    .DIA0(wdata[0]),
    .DIA1(wdata[1]),
    .DIA2(wdata[2]),
    .DIA3(wdata[3]),
    .DIA4(wdata[4]),
    .DIA5(wdata[5]),
    .DIA6(wdata[6]),
    .DIA7(wdata[7]),
    .DIB0(1'b0),
    .DIB1(1'b0),
    .DIB2(1'b0),
    .DIB3(1'b0),
    .DIB4(1'b0),
    .DIB5(1'b0),
    .DIB6(1'b0),
    .DIB7(1'b0),
    .DOB0(rdata_q[0]),
    .DOB1(rdata_q[1]),
    .DOB2(rdata_q[2]),
    .DOB3(rdata_q[3]),
    .DOB4(rdata_q[4]),
    .DOB5(rdata_q[5]),
    .DOB6(rdata_q[6]),
    .DOB7(rdata_q[7])
);

endmodule
