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

//========== reg ===========================
wire web;
wire rst_n;
wire wclk, rclk;
// addr
reg [$clog2(WORDS):0] w_addr, r_addr;
reg [$clog2(WORDS):0] w_addr_d, r_addr_d;

// pointer
reg [$clog2(WORDS):0] wptr;
reg [$clog2(WORDS):0] rptr;
wire [$clog2(WORDS):0] rq2_wptr;
wire [$clog2(WORDS):0] wq2_rptr;

// syn ip
NDFF_BUS_syn syn_r2w (.D(rptr), .Q(wq2_rptr), .clk(wclk), .rst_n(rst_n));
NDFF_BUS_syn syn_w2r (.D(wptr), .Q(rq2_wptr), .clk(rclk), .rst_n(rst_n));

// rdata
//  Add one more register stage to rdata
always @(posedge rclk, negedge rst_n) begin
    if (!rst_n) begin
        rdata <= 0;
    end
    else begin
		if (rinc & !rempty) begin
			rdata <= rdata_q;
		end
    end
end

//===== pointer =============================
always @(*) begin
  if(!rst_n)  rptr = 0;
  else        rptr = (r_addr >> 1)^r_addr;
end
always @(*) begin
  if(!rst_n)  wptr = 0;
  else        wptr = (w_addr >> 1)^w_addr;
end
//===========================================
//====== full & empty =======================
always @(posedge rclk or negedge rst_n)begin
	if(!rst_n) 	rempty <= 1'b1;
	else     	rempty <= (rptr == rq2_wptr);
end
always @(posedge wclk or negedge rst_n)begin
	if(!rst_n)  wfull <= 1'b0;
	else        wfull <= ((wptr[$clog2(WORDS)] != wq2_rptr[$clog2(WORDS)]) && (wptr[$clog2(WORDS) - 1] != wq2_rptr[$clog2(WORDS) - 1]) && (wptr[$clog2(WORDS) - 2 : 0] == wq2_rptr[$clog2(WORDS) - 2 : 0]));
end
//===========================================
//====== address ============================
always @(posedge wclk or negedge rst_n)begin
  if(!rst_n) 
      w_addr <= 0;
  else if ((wptr[$clog2(WORDS)] != wq2_rptr[$clog2(WORDS)]) && (wptr[$clog2(WORDS) - 1] != wq2_rptr[$clog2(WORDS) - 1]) && (wptr[$clog2(WORDS) - 2 : 0] == wq2_rptr[$clog2(WORDS) - 2 : 0]))
      w_addr <= w_addr;
  else if (winc && !wfull)
      w_addr <= w_addr + 1;
  else 
      w_addr <= w_addr;
end
always @(posedge rclk or negedge rst_n)begin
    if (!rst_n)
        r_addr <= 0;
    else if (rptr == rq2_wptr) 
        r_addr <= r_addr;
    else if (rinc && !rempty) 
        r_addr <= r_addr + 1;
    else 
        r_addr <= r_addr;
end

// web = 0 means write to SRAM, winc=1 and wfull!=1 we can write
  assign web = !(winc && (!wfull)); 

DUAL_64X8X1BM1 u_dual_sram (
    .CKA(wclk),
    .CKB(rclk),
    .WEAN(web),
    .WEBN(1'b1),
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
