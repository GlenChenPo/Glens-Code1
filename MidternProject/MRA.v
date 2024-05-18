//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Si2 LAB @NYCU ED430
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2024 Spring
//   Midterm Proejct            : MRA  
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : MRA.v
//   Module Name : MRA
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module MRA #(
  parameter ID_WIDTH   = 4,
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 128
) (
	// CHIP IO
	clk, rst_n,	in_valid,	frame_id,	net_id, loc_x, loc_y, cost, busy,
  // AXI4 IO
    // read address channel
	  arid_m_inf, araddr_m_inf, arlen_m_inf, arsize_m_inf, arburst_m_inf, arvalid_m_inf, arready_m_inf,
    // read data channel  
    rid_m_inf, rdata_m_inf, rresp_m_inf, rlast_m_inf, rvalid_m_inf, rready_m_inf,
	  // write address channel  
    awid_m_inf, awaddr_m_inf, awsize_m_inf, awburst_m_inf, awlen_m_inf, awvalid_m_inf, awready_m_inf,
    // write data channel
    wdata_m_inf, wlast_m_inf, wvalid_m_inf, wready_m_inf,
    // write response channel
      bid_m_inf, bresp_m_inf, bvalid_m_inf, bready_m_inf 
);
// ===============================================================
//  					Input / Output 
// ===============================================================
// << CHIP io port with system >>
input 			  	clk,rst_n;
input 			   	in_valid;
input  [4:0] 		frame_id;
input  [3:0]       	net_id;     
input  [5:0]       	loc_x; 
input  [5:0]       	loc_y; 
output reg [13:0] 	cost;
output reg          busy;       
  
// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
       Your AXI-4 interface could be designed as a bridge in submodule,
	   therefore I declared output of AXI as wire.  
	   Ex: AXI4_interface AXI4_INF(...);
*/

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output wire [1:0]            arburst_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [7:0]              arlen_m_inf;
output wire                  arvalid_m_inf;
input  wire                  arready_m_inf;
output wire [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output wire                   rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output reg [ID_WIDTH-1:0]      awid_m_inf;
output reg [1:0]            awburst_m_inf;
output reg [2:0]             awsize_m_inf;
output reg [7:0]              awlen_m_inf;
output reg                  awvalid_m_inf;
input  wire                  awready_m_inf;
output reg [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel 
output wire                   wvalid_m_inf;
input  wire                   wready_m_inf;
output wire [DATA_WIDTH-1:0]   wdata_m_inf;
output wire                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire  [ID_WIDTH-1:0]      bid_m_inf;
input  wire                   bvalid_m_inf;
output wire                   bready_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------

// ======================================================
//    reg & integer
// ======================================================
// reg for input -------------- reg for input //
reg in_valid_d;
reg [5:0] reg_loc_x_source[15:0];
reg [5:0] reg_loc_y_source[15:0];
reg [5:0] reg_loc_x_sink[15:0];
reg [5:0] reg_loc_y_sink[15:0];
reg [3:0] reg_net_id;
reg [4:0] reg_frame_id;
// reg for SRAM ---------------- reg for SRAM //
reg web_loc;
reg web_wei;
reg [8:0] addres_loc;
reg [7:0] addres_wei;
reg [127:0] DO_L;
reg [127:0] reg_DO_L;
reg [127:0] DO_W;
// reg for DRAM ---------------- reg for DRAM //
reg arvalid;
reg [ADDR_WIDTH-1:0]  araddr;
reg rready;
reg [DATA_WIDTH-1:0] SR_loc_in;
reg [DATA_WIDTH-1:0] rdata_wei;
reg [31:0] detector[127:0];
reg rlast_loc, rlast_wei;
// reg for DRAM write ---------------- reg for DRAM write //
// reg awvalid_m_inf;
reg wvalid;
reg wlast;
reg bready;
// reg [ADDR_WIDTH-1:0]  awaddr_m_inf;
reg [DATA_WIDTH-1:0]   wdata;


reg [5:0] curX,curY;
//  cnt  & flag
reg [15:0] cnt;
reg [15:0] cnt_d;
reg [5:0] map_cnt;
reg [5:0] cnt_2;
reg map_flag;
// reg [2:0] loc_map [63:0][63:0];
reg [2:0] loc_map [0:63][0:63];

reg flag_dram,flag_alldone;
reg flag;

// integer
integer i, j, l;
// dram signal ------------------------ dram signal //
assign arid_m_inf    = 0;
assign awid_m_inf    = 0;
assign arburst_m_inf = 2'b01;
assign awburst_m_inf = 2'b01;
assign arsize_m_inf  = 3'd4;
assign awsize_m_inf  = 3'd4;
assign arlen_m_inf   = 8'd127;
assign awlen_m_inf   = 8'd127;
assign araddr_m_inf = araddr;
assign arvalid_m_inf = arvalid;
assign rready_m_inf = rready;
// assign awvalid_m_inf = awvalid;
// assign awaddr_m_inf = awaddr;
assign wvalid_m_inf = wvalid;
assign wlast_m_inf = wlast;
assign wdata_m_inf = wdata;
assign bready_m_inf = bready;


// reg FSM -------------------------- reg FSM //
reg [5:0] state_cs, state_ns;
parameter S_IDLE          = 5'd0;
parameter S_LOAD          = 5'd1;
parameter S_rDRAM_LOC     = 5'd2;
parameter S_ARREADY_WEI   = 5'd3;
parameter S_READ_DRAM_WEI = 5'd4;
parameter S_MAP_LOC       = 5'd5;
parameter S_PRE_ROUTING   = 5'd6;
parameter S_ROUTING       = 5'd7;
parameter S_RETRACE       = 5'd8;
parameter S_TRANS_MAP     = 5'd9;
parameter S_SAVE_SRAM     = 5'd10;
// parameter  = 5'd11;
// parameter  = 5'd12;
// parameter  = 5'd13;
// parameter  = 5'd14;
parameter S_wDRAM        = 5'd15;
// ======================================================
//    STATE
// ======================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		state_cs <= S_IDLE;		
	end
	else state_cs <= state_ns;
end
always @(*) begin
	case (state_cs)
		S_IDLE : begin
			if (in_valid == 1) begin
				state_ns = S_LOAD;
			end
			else state_ns = S_IDLE;
		end
		S_LOAD : begin
			if (arready_m_inf) begin
				state_ns = S_rDRAM_LOC;
			end
			else state_ns = S_LOAD;
		end
		S_rDRAM_LOC : begin
			if (rlast_m_inf) begin
				state_ns = S_ARREADY_WEI;
			end
			else state_ns = S_rDRAM_LOC;
		end
    S_ARREADY_WEI : begin
			if (arready_m_inf) begin
				state_ns = S_READ_DRAM_WEI;
			end
			else state_ns = S_ARREADY_WEI;
    end
    S_READ_DRAM_WEI : begin
			if (rlast_m_inf) begin
				state_ns = S_MAP_LOC;
			end
			else state_ns = S_READ_DRAM_WEI;
    end
    S_MAP_LOC : begin
			if (!map_flag && addres_loc != 0) begin
				state_ns = S_PRE_ROUTING;
			end
			else state_ns = S_MAP_LOC;
    end
    S_PRE_ROUTING : begin // SET SINK and SOURCE to 6 and 5
			if (!map_flag) begin
				state_ns = S_ROUTING;
			end
			else state_ns = S_PRE_ROUTING;
    end
    S_ROUTING : begin  // set 123 123 123
      if (flag)  state_ns = S_RETRACE;
      else        state_ns = S_ROUTING;
    end
    S_RETRACE : begin
      if(loc_map[curY+1][curX]== 5 || loc_map[curY-1][curX]==5||loc_map[curY][curX+1]== 5 || loc_map[curY][curX-1]==5)   state_ns = S_SAVE_SRAM;
      else       state_ns = S_RETRACE;
		end
    S_SAVE_SRAM : begin
      if(flag_alldone) state_ns = S_wDRAM; 
    //   else if 
      else     state_ns = S_SAVE_SRAM;
    end    
    default: state_ns = state_ns;
	endcase
end
// ======================================================
//       INPUT
// ======================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_x_source[i] <= 0;
    end
	end
  else if ((in_valid) && cnt[0] == 0) begin
    reg_loc_x_source[cnt / 2] <= loc_x;
  end
	else begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_x_source[i] <= reg_loc_x_source[i];
    end
  end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_y_source[i] <= 0;
    end
	end
  else if ((in_valid) && cnt[0] == 0) begin
    reg_loc_y_source[cnt / 2] <= loc_y;
  end
	else begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_y_source[i] <= reg_loc_y_source[i];
    end
  end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_x_sink[i] <= 0;
    end
	end
  else if ((in_valid) && cnt[0] == 1) begin
    reg_loc_x_sink[cnt / 2] <= loc_x;
  end
	else begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_x_sink[i] <= reg_loc_x_sink[i];
    end
  end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_y_sink[i] <= 0;
    end
	end
  else if ((in_valid) && cnt[0] == 1) begin
    reg_loc_y_sink[cnt / 2] <= loc_y;
  end
	else begin
    for (i = 0; i < 16; i = i + 1) begin
      reg_loc_y_sink[i] <= reg_loc_y_sink[i];
    end
  end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		reg_net_id <= 0;
	end
	else if (in_valid == 1) begin
		reg_net_id <= net_id;
	end
	else reg_net_id <= reg_net_id;
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		reg_frame_id <= 0;
	end
	else if (in_valid == 1) begin
		reg_frame_id <= frame_id;
	end
	else reg_frame_id <= reg_frame_id;
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		in_valid_d <= 0;
	end
	else in_valid_d <= in_valid;
end
// ======================================================
//         counter
// ======================================================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cnt <= 0;
	end
	else if (state_cs == S_LOAD | in_valid) begin
		if (arvalid_m_inf == 1 && arready_m_inf == 1) begin
            cnt <= 0;
        end
        else cnt <= cnt + 1;
	end
	else if (state_cs == S_rDRAM_LOC) begin
        cnt <= cnt + (rready_m_inf == 1 && rvalid_m_inf == 1);
	end
	else if (state_cs == S_READ_DRAM_WEI) begin
		if (rlast_m_inf == 1) begin
			cnt <= 0;
		end
		else cnt <= cnt + 1;
	end
    else if (state_cs == S_MAP_LOC) begin
      cnt <= cnt + 1;
    end
    else if (state_cs == S_PRE_ROUTING) begin
      cnt <= 0;
    end
    else if (state_cs == S_ROUTING) begin
      if (cnt == 40) cnt <= 0;
      else cnt <= cnt + 1;
    end
    else if (state_cs == S_RETRACE) begin
      cnt <= 0;
    end
    else if (state_cs == S_SAVE_SRAM) begin
      cnt <= cnt + 1;
    end
	else cnt <= cnt;
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    cnt_d <= 0;
  end
  else cnt_d <= cnt;
end
// input data ----------------------------------------------- input data //
// design ------------------------------------------------------- design //
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		flag_dram <= 0;
	end
	else if (arready_m_inf == 1 && arvalid_m_inf == 1) begin
		flag_dram <= 1;
	end
	else if (rlast_m_inf == 1 && rready_m_inf == 1 && rvalid_m_inf == 1) begin
		flag_dram <= 0;
	end
	else flag_dram <= flag_dram;
end

// DRAM READ ADDRES ----------------------------------- DRAM READ ADDRES //
always @(*) begin
	if (!rst_n) begin
		arvalid = 0;
	end
	else if (state_cs == S_LOAD && in_valid_d == 0 || state_cs == S_ARREADY_WEI) begin
		arvalid = 1;
	end
	else arvalid = 0;
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) araddr <= 0;
	else if (!in_valid && in_valid_d) araddr <= {16'd1, reg_frame_id, 11'd0};
	else if (state_cs == S_rDRAM_LOC && rlast_m_inf == 1) araddr <= {16'd2, reg_frame_id, 11'd0};
	else araddr <= araddr;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		rready <= 0;
	end
	else if (arready_m_inf == 1 && arvalid_m_inf == 1) begin
		rready <= 1;
	end
	else if (rlast_m_inf == 1) begin
		rready <= 0;
	end
	else rready <= rready;
end

genvar k;
generate 
for (k = 0 ; k < 32 ; k  = k + 1) begin 
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 0; i < 128; i = i + 1) begin
        detector[i] <= 0;
      end
    end
    else if (state_cs == S_rDRAM_LOC) begin
      if (rvalid_m_inf && rready_m_inf) begin
        if (rdata_m_inf[(3 + (k * 4)):(0 + (k * 4))] != 0) begin
          detector[cnt][k] <= 1;
        end
        else detector[cnt][k] <= 0;
      end
      else begin
        for (i = 0;i < 128;i = i + 1) begin
          detector[i] <= 0;
        end
      end
    end
  end
end
endgenerate


// ======================================================
//          SRAM
// ======================================================
// web for location and weight ------------- web for location and weight //
// location loc_map ------------------------------- location loc_map //
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		web_loc <= 1;
	end
	else if (state_cs == S_rDRAM_LOC && rlast_m_inf == 0) begin
		if (rvalid_m_inf == 1) begin
			web_loc <= 0;
		end
		else web_loc <= 1;
	end
    else if (state_cs==S_SAVE_SRAM) begin
        if (addres_loc==127)  web_loc <= 1;
        else                  web_loc <= 0;
    end    
	else if (state_cs !== S_rDRAM_LOC) begin
		web_loc <= 1;
	end
	else web_loc <= web_loc;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		SR_loc_in <= 0;
	end
	else if (state_cs == S_rDRAM_LOC) begin
		SR_loc_in <= rdata_m_inf;
	end
    else if (state_cs == S_SAVE_SRAM)begin
      SR_loc_in <= {loc_map[map_cnt][cnt_2],loc_map[map_cnt][cnt_2+1],loc_map[map_cnt][cnt_2+2],loc_map[map_cnt][cnt_2+3],loc_map[map_cnt][cnt_2+4],loc_map[map_cnt][cnt_2+5],
      loc_map[map_cnt][cnt_2+6],loc_map[map_cnt][cnt_2+7],loc_map[map_cnt][cnt_2+8],loc_map[map_cnt][cnt_2+9],loc_map[map_cnt][cnt_2+10],loc_map[map_cnt][cnt_2+11],loc_map[map_cnt][cnt_2+12],
      loc_map[map_cnt][cnt_2+13],loc_map[map_cnt][cnt_2+14],loc_map[map_cnt][cnt_2+15],loc_map[map_cnt][cnt_2+16],loc_map[map_cnt][cnt_2+17],loc_map[map_cnt][cnt_2+18],loc_map[map_cnt][cnt_2+19],
      loc_map[map_cnt][cnt_2+20],loc_map[map_cnt][cnt_2+21],loc_map[map_cnt][cnt_2+22],loc_map[map_cnt][cnt_2+23],loc_map[map_cnt][cnt_2+24],loc_map[map_cnt][cnt_2+25],loc_map[map_cnt][cnt_2+26],
      loc_map[map_cnt][cnt_2+27],loc_map[map_cnt][cnt_2+28],loc_map[map_cnt][cnt_2+29],loc_map[map_cnt][cnt_2+30] };
    end
	else SR_loc_in <= 0;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		addres_loc <= 0;
	end
	else if (state_cs == S_rDRAM_LOC && web_loc == 0) begin
		addres_loc <= addres_loc + (rready_m_inf && rvalid_m_inf);
	end
    else if(state_cs == S_SAVE_SRAM&& web_loc == 0)begin
        addres_loc <= addres_loc+1;
    end
    else if (state_cs == S_MAP_LOC) begin
      if (addres_loc == 129) addres_loc <= 0;
      else addres_loc <= addres_loc + 1;
    end 
	else addres_loc <= 0;
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    reg_DO_L <= 0;
  end
  else if (state_cs == S_MAP_LOC) begin
    reg_DO_L <= DO_L;
  end
  else reg_DO_L <= reg_DO_L;
end

// weight loc_map ------------------------ weight loc_map //
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		web_wei <= 1;
	end
	else if (state_cs == S_READ_DRAM_WEI && rlast_m_inf == 0) begin
		if (rvalid_m_inf == 1) begin
			web_wei <= 0;
		end
		else web_wei <= 1;
	end
	else if (state_cs !== S_READ_DRAM_WEI) begin
		web_wei <= 1;
	end
	else web_wei <= web_wei;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		rdata_wei <= 0;
	end
	else if (state_cs == S_READ_DRAM_WEI) begin
		rdata_wei <= rdata_m_inf;
	end
	else rdata_wei <= 0;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		addres_wei <= 0;
	end
	else if (state_cs == S_READ_DRAM_WEI && web_wei == 0) begin
		addres_wei <= addres_wei + (rready_m_inf && rvalid_m_inf);
	end
	else addres_wei <= 0;
end
// ======================================================
//       DESIGN
// ======================================================
always @(posedge clk, negedge rst_n) begin
  if (!rst_n) map_flag <= 0;
  else if (state_cs == S_MAP_LOC) begin
    if (addres_loc == 129) map_flag <= 0;
  end
  else if (state_ns == S_MAP_LOC && addres_loc == 0) map_flag <= 1;
  else map_flag <= map_flag;
end
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    for (i = 0; i < 64; i = i + 1) begin
      for (j = 0; j < 64; j = j + 1) begin
        loc_map[i][j] <= 0;
      end
    end
  end
  else if (state_cs == S_MAP_LOC && (!cnt_d [0] && cnt[0])) begin
      loc_map[cnt_d >> 1][0] <=  (detector[cnt_d][0] << 2);
      loc_map[cnt_d >> 1][1] <=  (detector[cnt_d][1] << 2);
      loc_map[cnt_d >> 1][2] <=  (detector[cnt_d][2] << 2);
      loc_map[cnt_d >> 1][3] <=  (detector[cnt_d][3] << 2);
      loc_map[cnt_d >> 1][4] <=  (detector[cnt_d][4] << 2);
      loc_map[cnt_d >> 1][5] <=  (detector[cnt_d][5] << 2);
      loc_map[cnt_d >> 1][6] <=  (detector[cnt_d][6] << 2);
      loc_map[cnt_d >> 1][7] <=  (detector[cnt_d][7] << 2);
      loc_map[cnt_d >> 1][8] <=  (detector[cnt_d][8] << 2);
      loc_map[cnt_d >> 1][9] <=  (detector[cnt_d][9] << 2);
      loc_map[cnt_d >> 1][10] <= (detector[cnt_d][10] << 2);
      loc_map[cnt_d >> 1][11] <= (detector[cnt_d][11] << 2);
      loc_map[cnt_d >> 1][12] <= (detector[cnt_d][12] << 2);
      loc_map[cnt_d >> 1][13] <= (detector[cnt_d][13] << 2);
      loc_map[cnt_d >> 1][14] <= (detector[cnt_d][14] << 2);
      loc_map[cnt_d >> 1][15] <= (detector[cnt_d][15] << 2);
      loc_map[cnt_d >> 1][16] <= (detector[cnt_d][16] << 2);
      loc_map[cnt_d >> 1][17] <= (detector[cnt_d][17] << 2);
      loc_map[cnt_d >> 1][18] <= (detector[cnt_d][18] << 2);
      loc_map[cnt_d >> 1][19] <= (detector[cnt_d][19] << 2);
      loc_map[cnt_d >> 1][20] <= (detector[cnt_d][20] << 2);
      loc_map[cnt_d >> 1][21] <= (detector[cnt_d][21] << 2);
      loc_map[cnt_d >> 1][22] <= (detector[cnt_d][22] << 2);
      loc_map[cnt_d >> 1][23] <= (detector[cnt_d][23] << 2);
      loc_map[cnt_d >> 1][24] <= (detector[cnt_d][24] << 2);
      loc_map[cnt_d >> 1][25] <= (detector[cnt_d][25] << 2);
      loc_map[cnt_d >> 1][26] <= (detector[cnt_d][26] << 2);
      loc_map[cnt_d >> 1][27] <= (detector[cnt_d][27] << 2);
      loc_map[cnt_d >> 1][28] <= (detector[cnt_d][28] << 2);
      loc_map[cnt_d >> 1][29] <= (detector[cnt_d][29] << 2);
      loc_map[cnt_d >> 1][30] <= (detector[cnt_d][30] << 2);
      loc_map[cnt_d >> 1][31] <= (detector[cnt_d][31] << 2);
      loc_map[cnt_d >> 1][32] <= (detector[cnt][0] << 2);
      loc_map[cnt_d >> 1][33] <= (detector[cnt][1] << 2);
      loc_map[cnt_d >> 1][34] <= (detector[cnt][2] << 2);
      loc_map[cnt_d >> 1][35] <= (detector[cnt][3] << 2);
      loc_map[cnt_d >> 1][36] <= (detector[cnt][4] << 2);
      loc_map[cnt_d >> 1][37] <= (detector[cnt][5] << 2);
      loc_map[cnt_d >> 1][38] <= (detector[cnt][6] << 2);
      loc_map[cnt_d >> 1][39] <= (detector[cnt][7] << 2);
      loc_map[cnt_d >> 1][40] <= (detector[cnt][8] << 2);
      loc_map[cnt_d >> 1][41] <= (detector[cnt][9] << 2);
      loc_map[cnt_d >> 1][42] <= (detector[cnt][10] << 2);
      loc_map[cnt_d >> 1][43] <= (detector[cnt][11] << 2);
      loc_map[cnt_d >> 1][44] <= (detector[cnt][12] << 2);
      loc_map[cnt_d >> 1][45] <= (detector[cnt][13] << 2);
      loc_map[cnt_d >> 1][46] <= (detector[cnt][14] << 2);
      loc_map[cnt_d >> 1][47] <= (detector[cnt][15] << 2);
      loc_map[cnt_d >> 1][48] <= (detector[cnt][16] << 2);
      loc_map[cnt_d >> 1][49] <= (detector[cnt][17] << 2);
      loc_map[cnt_d >> 1][50] <= (detector[cnt][18] << 2);
      loc_map[cnt_d >> 1][51] <= (detector[cnt][19] << 2);
      loc_map[cnt_d >> 1][52] <= (detector[cnt][20] << 2);
      loc_map[cnt_d >> 1][53] <= (detector[cnt][21] << 2);
      loc_map[cnt_d >> 1][54] <= (detector[cnt][22] << 2);
      loc_map[cnt_d >> 1][55] <= (detector[cnt][23] << 2);
      loc_map[cnt_d >> 1][56] <= (detector[cnt][24] << 2);
      loc_map[cnt_d >> 1][57] <= (detector[cnt][25] << 2);
      loc_map[cnt_d >> 1][58] <= (detector[cnt][26] << 2);
      loc_map[cnt_d >> 1][59] <= (detector[cnt][27] << 2);
      loc_map[cnt_d >> 1][60] <= (detector[cnt][28] << 2);
      loc_map[cnt_d >> 1][61] <= (detector[cnt][29] << 2);
      loc_map[cnt_d >> 1][62] <= (detector[cnt][30] << 2);
      loc_map[cnt_d >> 1][63] <= (detector[cnt][31] << 2);
  end
  else if (state_cs == S_PRE_ROUTING) begin
    loc_map[reg_loc_y_source[0]][reg_loc_x_source[0]] <= 5;
    loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]] <= 6;
  end
  else if (state_cs == S_ROUTING) begin
    for (i = 0; i < 64; i = i + 1) begin        
        for (j = 0; j < 64; j = j + 1) begin
            if (loc_map[i][j] == 5 && map_cnt == 0) begin
              if (loc_map[i+1][j]==0)begin  // down = 0
                  if (loc_map[i-1][j]==0)begin  // up = 0
                      if (loc_map[i][j-1]==0)begin  // left = 0
                          if (loc_map[i][j+1]==0)begin  // right = 0
                            loc_map[i][j+1] <= 1;
                            loc_map[i][j-1] <= 1;
                            loc_map[i+1][j] <= 1;
                            loc_map[i-1][j] <= 1;
                          end    
                          else begin               // right != 0
                            loc_map[i][j-1] <= 1;
                            loc_map[i+1][j] <= 1;
                            loc_map[i-1][j] <= 1;
                          end
                      end
                      else begin              // left != 0
                          if (loc_map[i][j+1]==0)begin  // right = 0
                            loc_map[i+1][j] <= 1;
                            loc_map[i-1][j] <= 1;
                            loc_map[i][j+1] <= 1;        
                          end    
                          else begin                // right != 0
                            loc_map[i+1][j] <= 1;
                            loc_map[i-1][j] <= 1;
                          end
                      end
                  end
                  else begin              // up != 0
                      if (loc_map[i][j-1]==0)begin // left = 0
                          if (loc_map[i][j+1]==0)begin // right = 0
                            loc_map[i][j+1] <= 1;
                            loc_map[i][j-1] <= 1;
                            loc_map[i+1][j] <= 1;
                          end    
                          else begin               // right != 0
                            loc_map[i][j-1] <= 1;
                            loc_map[i+1][j] <= 1;
                          end
                      end
                      else begin               // left != 0
                          if (loc_map[i][j+1]==0)begin // right = 0
                            loc_map[i][j+1] <= 1;
                            loc_map[i+1][j] <= 1;
                          end    
                          else begin               // right != 0
                            loc_map[i+1][j] <= 1;
                          end
                      end    
                  end
              end
              else begin // down != 0
                  if (loc_map[i-1][j]==0)begin  // up = 0
                      if (loc_map[i][j-1]==0)begin  // left = 0
                          if (loc_map[i][j+1]==0)begin  // right = 0
                            loc_map[i][j+1] <= 1;
                            loc_map[i][j-1] <= 1;
                            loc_map[i-1][j] <= 1;
                          end    
                          else begin               // right != 0
                            loc_map[i][j-1] <= 1;
                            loc_map[i-1][j] <= 1;
                          end
                      end
                      else begin              // left != 0
                          if (loc_map[i][j+1]==0)begin  // right = 0
                            loc_map[i-1][j] <= 1;
                            loc_map[i][j+1] <= 1;        
                          end    
                          else begin                // right != 0
                            loc_map[i-1][j] <= 1;
                          end
                      end
                  end
                  else begin              // up != 0
                      if (loc_map[i][j-1]==0)begin // left = 0
                          if (loc_map[i][j+1]==0)begin // right = 0
                            loc_map[i][j+1] <= 1;
                            loc_map[i][j-1] <= 1;
                          end    
                          else begin               // right != 0
                            loc_map[i][j-1] <= 1;
                          end
                      end
                      else begin               // left != 0
                          if (loc_map[i][j+1]==0)begin // right = 0
                            loc_map[i][j+1] <= 1;
                          end    
                          else begin               // right != 0
                            //empty 
                            // can del this else because u don;t need to do anything
                          end
                      end    
                  end
              end
            end
            else if (loc_map[i][j] == 1 && map_cnt == 1) begin
                if (loc_map[i+1][j]==0)begin  // down = 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 2;
                              loc_map[i][j-1] <= 2;
                              loc_map[i+1][j] <= 2;
                              loc_map[i-1][j] <= 2;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 2;
                              loc_map[i+1][j] <= 2;
                              loc_map[i-1][j] <= 2;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i+1][j] <= 2;
                              loc_map[i-1][j] <= 2;
                              loc_map[i][j+1] <= 2;        
                            end    
                            else begin                // right != 0
                              loc_map[i+1][j] <= 2;
                              loc_map[i-1][j] <= 2;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 2;
                              loc_map[i][j-1] <= 2;
                              loc_map[i+1][j] <= 2;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 2;
                              loc_map[i+1][j] <= 2;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 2;
                              loc_map[i+1][j] <= 2;
                            end    
                            else begin               // right != 0
                              loc_map[i+1][j] <= 2;
                            end
                        end    
                    end
                end
                else begin // down != 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 2;
                              loc_map[i][j-1] <= 2;
                              loc_map[i-1][j] <= 2;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 2;
                              loc_map[i-1][j] <= 2;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i-1][j] <= 2;
                              loc_map[i][j+1] <= 2;        
                            end    
                            else begin                // right != 0
                              loc_map[i-1][j] <= 2;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 2;
                              loc_map[i][j-1] <= 2;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 2;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 2;
                            end    
                        end    
                    end
                end            
            end
            else if (loc_map[i][j] == 2 && map_cnt == 2) begin
                if (loc_map[i+1][j]==0)begin  // down = 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 3;
                              loc_map[i][j-1] <= 3;
                              loc_map[i+1][j] <= 3;
                              loc_map[i-1][j] <= 3;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 3;
                              loc_map[i+1][j] <= 3;
                              loc_map[i-1][j] <= 3;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i+1][j] <= 3;
                              loc_map[i-1][j] <= 3;
                              loc_map[i][j+1] <= 3;        
                            end    
                            else begin                // right != 0
                              loc_map[i+1][j] <= 3;
                              loc_map[i-1][j] <= 3;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 3;
                              loc_map[i][j-1] <= 3;
                              loc_map[i+1][j] <= 3;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 3;
                              loc_map[i+1][j] <= 3;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 3;
                              loc_map[i+1][j] <= 3;
                            end    
                            else begin               // right != 0
                              loc_map[i+1][j] <= 3;
                            end
                        end    
                    end
                end
                else begin // down != 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 3;
                              loc_map[i][j-1] <= 3;
                              loc_map[i-1][j] <= 3;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 3;
                              loc_map[i-1][j] <= 3;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i-1][j] <= 3;
                              loc_map[i][j+1] <= 3;        
                            end    
                            else begin                // right != 0
                              loc_map[i-1][j] <= 3;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 3;
                              loc_map[i][j-1] <= 3;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 3;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 3;
                            end    
                        end    
                    end
                end            
            end        
            else if (loc_map[i][j] == 3 && map_cnt == 3) begin
                if (loc_map[i+1][j]==0)begin  // down = 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 1;
                              loc_map[i][j-1] <= 1;
                              loc_map[i+1][j] <= 1;
                              loc_map[i-1][j] <= 1;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 1;
                              loc_map[i+1][j] <= 1;
                              loc_map[i-1][j] <= 1;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i+1][j] <= 1;
                              loc_map[i-1][j] <= 1;
                              loc_map[i][j+1] <= 1;        
                            end    
                            else begin                // right != 0
                              loc_map[i+1][j] <= 1;
                              loc_map[i-1][j] <= 1;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 1;
                              loc_map[i][j-1] <= 1;
                              loc_map[i+1][j] <= 1;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 1;
                              loc_map[i+1][j] <= 1;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 1;
                              loc_map[i+1][j] <= 1;
                            end    
                            else begin               // right != 0
                              loc_map[i+1][j] <= 1;
                            end
                        end    
                    end
                end
                else begin // down != 0
                    if (loc_map[i-1][j]==0)begin  // up = 0
                        if (loc_map[i][j-1]==0)begin  // left = 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i][j+1] <= 1;
                              loc_map[i][j-1] <= 1;
                              loc_map[i-1][j] <= 1;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 1;
                              loc_map[i-1][j] <= 1;
                            end
                        end
                        else begin              // left != 0
                            if (loc_map[i][j+1]==0)begin  // right = 0
                              loc_map[i-1][j] <= 1;
                              loc_map[i][j+1] <= 1;        
                            end    
                            else begin                // right != 0
                              loc_map[i-1][j] <= 1;
                            end
                        end
                    end
                    else begin              // up != 0
                        if (loc_map[i][j-1]==0)begin // left = 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 1;
                              loc_map[i][j-1] <= 1;
                            end    
                            else begin               // right != 0
                              loc_map[i][j-1] <= 1;
                            end
                        end
                        else begin               // left != 0
                            if (loc_map[i][j+1]==0)begin // right = 0
                              loc_map[i][j+1] <= 1;
                            end    
                        end    
                    end
                end            
            end         
        end
      end
    end
  else if (state_cs == S_RETRACE)begin
    if (loc_map[curY][curX]!=6)
    // if(loc_map[curY+1][curX]!= 5 || loc_map[curY-1][curX]!=5||loc_map[curY][curX+1]!= 5 || loc_map[curY][curX-1]!=5)
      loc_map[curY][curX] <= 7;
  end  
end
always @(posedge clk or negedge rst_n) begin //Priority DOWN UP RIGHT LEFT
  if(!rst_n) begin
    curX <= 0;
    curY <= 0;
  end
  else if(state_cs==S_ROUTING)begin
    curX <= reg_loc_x_sink[0];
    curY <= reg_loc_y_sink[0];
  end
  else if(state_cs==S_RETRACE)begin
    if (curY > reg_loc_y_source[0]) 
    begin //sink under source
        if (curX > reg_loc_x_source[0]) 
        begin //sink righter source      up left down right
          if(loc_map[curY-1][curX]==map_cnt)      curY <= curY-1;
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1;
          else if(loc_map[curY+1][curX]==map_cnt) curY <= curY+1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX-1;
        end
        else if (curX < reg_loc_x_source[0]) 
        begin //sink lefter source       up right down left
          if(loc_map[curY-1][curX]==map_cnt)      curY <= curY-1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX+1;
          else if(loc_map[curY+1][curX]==map_cnt) curY <= curY+1;
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1;        
        end
        else  // curX == reg_loc_x_source 
        begin
          if(loc_map[curY-1][curX]==map_cnt)      curY <= curY-1;
          else if(loc_map[curY+1][curX]==map_cnt) curY <= curY+1;
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1;          
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX-1;
        end
    end
    else if (curY > reg_loc_y_source[0])
    begin // sink upper source
        if (curX > reg_loc_x_source[0]) 
        begin //sink righter source   
          if(loc_map[curY+1][curX]==map_cnt)      curY <= curY+1;
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1;
          else if(loc_map[curY-1][curX]==map_cnt) curY <= curY-1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX+1; 
        end
        else if (curX < reg_loc_x_source[0])
        begin //sink lefter source  
          if(loc_map[curY+1][curX]==map_cnt)      curY <= curY+1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX+1;
          else if(loc_map[curY-1][curX]==map_cnt) curY <= curY-1;
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1; 
        end      
        else  // curX == reg_loc_x_source 
        begin
          if(loc_map[curY+1][curX]==map_cnt)      curY <= curY+1;
          else if(loc_map[curY-1][curX]==map_cnt) curY <= curY-1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX+1;          
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1; 
        end        
    end
    else  // curY == reg_loc_y_source
    begin 
        if (curX > reg_loc_x_source[0]) 
        begin //sink righter source   
          if(loc_map[curY][curX-1]==map_cnt)      curX <= curX-1;
          else if(loc_map[curY+1][curX]==map_cnt) curY <= curY+1;
          else if(loc_map[curY-1][curX]==map_cnt) curY <= curY-1;
          else if(loc_map[curY][curX+1]==map_cnt) curX <= curX+1;          
        end
        else if (curX < reg_loc_x_source[0])
        begin //sink lefter source  
          if (loc_map[curY][curX+1]==map_cnt)     curX <= curX+1;        
          else if(loc_map[curY+1][curX]==map_cnt) curY <= curY+1;
          else if(loc_map[curY-1][curX]==map_cnt) curY <= curY-1; 
          else if(loc_map[curY][curX-1]==map_cnt) curX <= curX-1;          
        end      
    end
  end
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) flag <= 0;
  else if (state_cs ==S_ROUTING) begin
    if (flag) 
     flag <= 0;
    else if(loc_map[reg_loc_y_sink[0]+1][reg_loc_x_sink[0]]!=0 && loc_map[reg_loc_y_sink[0]-1][reg_loc_x_sink[0]]!=0&&loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]-1]!=0&&loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]+1]!=0)
     flag <= 1;
  end
  else if (state_cs ==S_RETRACE) begin
    if(loc_map[reg_loc_y_source[0]+1][reg_loc_x_source[0]]==7 || loc_map[reg_loc_y_source[0]-1][reg_loc_x_source[0]]==7||loc_map[reg_loc_y_source[0]][reg_loc_x_source[0]-1]==7||loc_map[reg_loc_y_source[0]][reg_loc_x_source[0]+1]==7)
     flag <= 1;
  end
  else flag <= 0; 
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    map_cnt <= 0;
  end
  else if (state_cs == S_ROUTING) begin
    if (flag)  begin
      if(loc_map[reg_loc_y_sink[0]+1][reg_loc_x_sink[0]]==1 || loc_map[reg_loc_y_sink[0]-1][reg_loc_x_sink[0]]==1||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]-1]==1||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]+1]==1)
        map_cnt <= 1;
      else if(loc_map[reg_loc_y_sink[0]+1][reg_loc_x_sink[0]]==2 || loc_map[reg_loc_y_sink[0]-1][reg_loc_x_sink[0]]==2||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]-1]==2||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]+1]==2)
        map_cnt <= 2;        
      else if(loc_map[reg_loc_y_sink[0]+1][reg_loc_x_sink[0]]==3 || loc_map[reg_loc_y_sink[0]-1][reg_loc_x_sink[0]]==3||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]-1]==3||loc_map[reg_loc_y_sink[0]][reg_loc_x_sink[0]+1]==3)
        map_cnt <= 3;
    end
    else if (map_cnt == 3)  map_cnt <= 1;
    else map_cnt <= map_cnt + 1;
  end
  else if (state_cs==S_SAVE_SRAM)begin
    if(cnt_2==31) map_cnt <= map_cnt+1;
    else        map_cnt <= map_cnt;
  end  
  else if (state_ns == S_SAVE_SRAM)begin
     map_cnt <= 0;
  end
  else if (state_cs == S_RETRACE)begin
    if (map_cnt == 1)  map_cnt <= 3;
    else map_cnt <= map_cnt - 1;
  end

end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    cnt_2 <= 0;
  end
  else if(state_cs==S_SAVE_SRAM)begin
    if(cnt_2==31) cnt_2<=0;
    else         cnt_2<=31;
  end
end
// output reset ------------------------------------------- output reset //
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		cost <= 0;
	end
	else cost <= cost;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		busy <= 0;
	end
	else if (arready_m_inf == 1) begin
		busy <= 1;
	end
	else if (bvalid_m_inf == 1) begin
		busy <= 0;
	end
	else busy <= busy;
end
// output reset ------------------------------------------- output reset //
// output wirte to dram to check --------- output wirte to dram to check //
// awvalid
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    awvalid_m_inf <= 0;
  end
  else if (state_cs == S_SAVE_SRAM) begin
    if (cnt==0) awvalid_m_inf <= 1;
    else if (awready_m_inf) awvalid_m_inf <= 0;
  end
  else awvalid_m_inf <= awvalid_m_inf;
end
// awaddr
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    awaddr_m_inf <= 0;
  end
  else if (state_cs == S_RETRACE) begin // at output state awadder is the addr where i read
    awaddr_m_inf <= {16'd1, reg_frame_id, 11'd0};
  end
end
// wvalid
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wvalid <= 0;
  end
  else if (state_cs == S_SAVE_SRAM) begin
    if (awready_m_inf) 
        wvalid <= 1;
    else if(wvalid_m_inf)
        wvalid <= 0; 
  end
  else wvalid <= 0;
end
// WLAST
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wlast <= 0;
  end
  else wlast <= wlast;
end
// wdata 
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wdata <= 0;
  end
  else wdata <= 1;
end
// bready
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    bready <= 0;
  end
  else if (state_cs == S_SAVE_SRAM) begin
    if (cnt == 0 || cnt==1) begin //first two cnt awvalid == 1
      bready <= 0;
    end
    else bready <= 1; //else cnt awvalid == 0
  end
  else bready <= bready;
end
// output wirte to dram to check --------- output wirte to dram to check //

// location sram ----------------------------------------- location sram //
  SUMA180_128X128X1BM1 LOCATION_MAP (
    .A0(addres_loc[0]),.A1(addres_loc[1]),.A2(addres_loc[2]),.A3(addres_loc[3]),.A4(addres_loc[4]),.A5(addres_loc[5]),.A6(addres_loc[6]),
    .DO0(DO_L[0]),.DO1(DO_L[1]),.DO2(DO_L[2]),.DO3(DO_L[3]),.DO4(DO_L[4]),.DO5(DO_L[5]),.DO6(DO_L[6]),.DO7(DO_L[7]),.DO8(DO_L[8]),.DO9(DO_L[9]),.DO10(DO_L[10]),
    .DO11(DO_L[11]),.DO12(DO_L[12]),.DO13(DO_L[13]),.DO14(DO_L[14]),.DO15(DO_L[15]),.DO16(DO_L[16]),.DO17(DO_L[17]),.DO18(DO_L[18]),.DO19(DO_L[19]),.DO20(DO_L[20]),
    .DO21(DO_L[21]),.DO22(DO_L[22]),.DO23(DO_L[23]),.DO24(DO_L[24]),.DO25(DO_L[25]),.DO26(DO_L[26]),.DO27(DO_L[27]),.DO28(DO_L[28]),.DO29(DO_L[29]),.DO30(DO_L[30]),
    .DO31(DO_L[31]),.DO32(DO_L[32]),.DO33(DO_L[33]),.DO34(DO_L[34]),.DO35(DO_L[35]),.DO36(DO_L[36]),.DO37(DO_L[37]),.DO38(DO_L[38]),.DO39(DO_L[39]),.DO40(DO_L[40]),
    .DO41(DO_L[41]),.DO42(DO_L[42]),.DO43(DO_L[43]),.DO44(DO_L[44]),.DO45(DO_L[45]),.DO46(DO_L[46]),.DO47(DO_L[47]),.DO48(DO_L[48]),.DO49(DO_L[49]),.DO50(DO_L[50]),
    .DO51(DO_L[51]),.DO52(DO_L[52]),.DO53(DO_L[53]),.DO54(DO_L[54]),.DO55(DO_L[55]),.DO56(DO_L[56]),.DO57(DO_L[57]),.DO58(DO_L[58]),.DO59(DO_L[59]),.DO60(DO_L[60]),
    .DO61(DO_L[61]),.DO62(DO_L[62]),.DO63(DO_L[63]),.DO64(DO_L[64]),.DO65(DO_L[65]),.DO66(DO_L[66]),.DO67(DO_L[67]),.DO68(DO_L[68]),.DO69(DO_L[69]),.DO70(DO_L[70]),
    .DO71(DO_L[71]),.DO72(DO_L[72]),.DO73(DO_L[73]),.DO74(DO_L[74]),.DO75(DO_L[75]),.DO76(DO_L[76]),.DO77(DO_L[77]),.DO78(DO_L[78]),.DO79(DO_L[79]),.DO80(DO_L[80]),
    .DO81(DO_L[81]),.DO82(DO_L[82]),.DO83(DO_L[83]),.DO84(DO_L[84]),.DO85(DO_L[85]),.DO86(DO_L[86]),.DO87(DO_L[87]),.DO88(DO_L[88]),.DO89(DO_L[89]),.DO90(DO_L[90]),
    .DO91(DO_L[91]),.DO92(DO_L[92]),.DO93(DO_L[93]),.DO94(DO_L[94]),.DO95(DO_L[95]),.DO96(DO_L[96]),.DO97(DO_L[97]),.DO98(DO_L[98]),.DO99(DO_L[99]),.DO100(DO_L[100]),
    .DO101(DO_L[101]),.DO102(DO_L[102]),.DO103(DO_L[103]),.DO104(DO_L[104]),.DO105(DO_L[105]),.DO106(DO_L[106]),.DO107(DO_L[107]),.DO108(DO_L[108]),.DO109(DO_L[109]),.DO110(DO_L[110]),
    .DO111(DO_L[111]),.DO112(DO_L[112]),.DO113(DO_L[113]),.DO114(DO_L[114]),.DO115(DO_L[115]),.DO116(DO_L[116]),.DO117(DO_L[117]),.DO118(DO_L[118]),.DO119(DO_L[119]),.DO120(DO_L[120]),
    .DO121(DO_L[121]),.DO122(DO_L[122]),.DO123(DO_L[123]),.DO124(DO_L[124]),.DO125(DO_L[125]),.DO126(DO_L[126]),.DO127(DO_L[127]),
    .DI0  (SR_loc_in[0]),.DI1(SR_loc_in[1]),.DI2(SR_loc_in[2]),.DI3(SR_loc_in[3]),.DI4(SR_loc_in[4]),.DI5(SR_loc_in[5]),.DI6(SR_loc_in[6]),.DI7(SR_loc_in[7]),.DI8(SR_loc_in[8]),.DI9(SR_loc_in[9]),.DI10(SR_loc_in[10]),
    .DI11 (SR_loc_in[11]),.DI12 (SR_loc_in[12]),.DI13 (SR_loc_in[13]),.DI14 (SR_loc_in[14]),.DI15 (SR_loc_in[15]),.DI16 (SR_loc_in[16]),.DI17 (SR_loc_in[17]),.DI18 (SR_loc_in[18]),.DI19 (SR_loc_in[19]),.DI20 (SR_loc_in[20]),
    .DI21 (SR_loc_in[21]),.DI22 (SR_loc_in[22]),.DI23 (SR_loc_in[23]),.DI24 (SR_loc_in[24]),.DI25 (SR_loc_in[25]),.DI26 (SR_loc_in[26]),.DI27 (SR_loc_in[27]),.DI28 (SR_loc_in[28]),.DI29 (SR_loc_in[29]),.DI30 (SR_loc_in[30]),
    .DI31 (SR_loc_in[31]),.DI32 (SR_loc_in[32]),.DI33 (SR_loc_in[33]),.DI34 (SR_loc_in[34]),.DI35 (SR_loc_in[35]),.DI36 (SR_loc_in[36]),.DI37 (SR_loc_in[37]),.DI38 (SR_loc_in[38]),.DI39 (SR_loc_in[39]),.DI40 (SR_loc_in[40]),
    .DI41 (SR_loc_in[41]),.DI42 (SR_loc_in[42]),.DI43 (SR_loc_in[43]),.DI44 (SR_loc_in[44]),.DI45 (SR_loc_in[45]),.DI46 (SR_loc_in[46]),.DI47 (SR_loc_in[47]),.DI48 (SR_loc_in[48]),.DI49 (SR_loc_in[49]),.DI50 (SR_loc_in[50]),
    .DI51 (SR_loc_in[51]),.DI52 (SR_loc_in[52]),.DI53 (SR_loc_in[53]),.DI54 (SR_loc_in[54]),.DI55 (SR_loc_in[55]),.DI56 (SR_loc_in[56]),.DI57 (SR_loc_in[57]),.DI58 (SR_loc_in[58]),.DI59 (SR_loc_in[59]),.DI60 (SR_loc_in[60]),
    .DI61 (SR_loc_in[61]),.DI62 (SR_loc_in[62]),.DI63 (SR_loc_in[63]),.DI64 (SR_loc_in[64]),.DI65 (SR_loc_in[65]),.DI66 (SR_loc_in[66]),.DI67 (SR_loc_in[67]),.DI68 (SR_loc_in[68]),.DI69 (SR_loc_in[69]),.DI70 (SR_loc_in[70]),
    .DI71 (SR_loc_in[71]),.DI72 (SR_loc_in[72]),.DI73 (SR_loc_in[73]),.DI74 (SR_loc_in[74]),.DI75 (SR_loc_in[75]),.DI76 (SR_loc_in[76]),.DI77 (SR_loc_in[77]),.DI78 (SR_loc_in[78]),.DI79 (SR_loc_in[79]),.DI80 (SR_loc_in[80]),
    .DI81 (SR_loc_in[81]),.DI82 (SR_loc_in[82]),.DI83 (SR_loc_in[83]),.DI84 (SR_loc_in[84]),.DI85 (SR_loc_in[85]),.DI86 (SR_loc_in[86]),.DI87 (SR_loc_in[87]),.DI88 (SR_loc_in[88]),.DI89 (SR_loc_in[89]),.DI90 (SR_loc_in[90]),
    .DI91 (SR_loc_in[91]),.DI92 (SR_loc_in[92]),.DI93 (SR_loc_in[93]),.DI94 (SR_loc_in[94]),.DI95 (SR_loc_in[95]),.DI96 (SR_loc_in[96]),.DI97 (SR_loc_in[97]),.DI98 (SR_loc_in[98]),.DI99 (SR_loc_in[99]),.DI100(SR_loc_in[100]),
    .DI101(SR_loc_in[101]),.DI102(SR_loc_in[102]),.DI103(SR_loc_in[103]),.DI104(SR_loc_in[104]),.DI105(SR_loc_in[105]),.DI106(SR_loc_in[106]),.DI107(SR_loc_in[107]),.DI108(SR_loc_in[108]),.DI109(SR_loc_in[109]),.DI110(SR_loc_in[110]),
    .DI111(SR_loc_in[111]),.DI112(SR_loc_in[112]),.DI113(SR_loc_in[113]),.DI114(SR_loc_in[114]),.DI115(SR_loc_in[115]),.DI116(SR_loc_in[116]),.DI117(SR_loc_in[117]),.DI118(SR_loc_in[118]),.DI119(SR_loc_in[119]),.DI120(SR_loc_in[120]),
    .DI121(SR_loc_in[121]),.DI122(SR_loc_in[122]),.DI123(SR_loc_in[123]),.DI124(SR_loc_in[124]),.DI125(SR_loc_in[125]),.DI126(SR_loc_in[126]),.DI127(SR_loc_in[127]),
    .CK(clk),
    .WEB(web_loc),
    .OE(1'b1),
    .CS(1'b1));

  SUMA180_128X128X1BM1 WEIGHT_MAP (
    .A0    (addres_wei[0]),
    .A1    (addres_wei[1]),
    .A2    (addres_wei[2]),
    .A3    (addres_wei[3]),
    .A4    (addres_wei[4]),
    .A5    (addres_wei[5]),
    .A6    (addres_wei[6]),
    .DI0   (rdata_wei[0]),
    .DI1   (rdata_wei[1]),
    .DI2   (rdata_wei[2]),
    .DI3   (rdata_wei[3]),
    .DI4   (rdata_wei[4]),
    .DI5   (rdata_wei[5]),
    .DI6   (rdata_wei[6]),
    .DI7   (rdata_wei[7]),
    .DI8   (rdata_wei[8]),
    .DI9   (rdata_wei[9]),
    .DI10  (rdata_wei[10]),
    .DI11  (rdata_wei[11]),
    .DI12  (rdata_wei[12]),
    .DI13  (rdata_wei[13]),
    .DI14  (rdata_wei[14]),
    .DI15  (rdata_wei[15]),
    .DI16  (rdata_wei[16]),
    .DI17  (rdata_wei[17]),
    .DI18  (rdata_wei[18]),
    .DI19  (rdata_wei[19]),
    .DI20  (rdata_wei[20]),
    .DI21  (rdata_wei[21]),
    .DI22  (rdata_wei[22]),
    .DI23  (rdata_wei[23]),
    .DI24  (rdata_wei[24]),
    .DI25  (rdata_wei[25]),
    .DI26  (rdata_wei[26]),
    .DI27  (rdata_wei[27]),
    .DI28  (rdata_wei[28]),
    .DI29  (rdata_wei[29]),
    .DI30  (rdata_wei[30]),
    .DI31  (rdata_wei[31]),
    .DI32  (rdata_wei[32]),
    .DI33  (rdata_wei[33]),
    .DI34  (rdata_wei[34]),
    .DI35  (rdata_wei[35]),
    .DI36  (rdata_wei[36]),
    .DI37  (rdata_wei[37]),
    .DI38  (rdata_wei[38]),
    .DI39  (rdata_wei[39]),
    .DI40  (rdata_wei[40]),
    .DI41  (rdata_wei[41]),
    .DI42  (rdata_wei[42]),
    .DI43  (rdata_wei[43]),
    .DI44  (rdata_wei[44]),
    .DI45  (rdata_wei[45]),
    .DI46  (rdata_wei[46]),
    .DI47  (rdata_wei[47]),
    .DI48  (rdata_wei[48]),
    .DI49  (rdata_wei[49]),
    .DI50  (rdata_wei[50]),
    .DI51  (rdata_wei[51]),
    .DI52  (rdata_wei[52]),
    .DI53  (rdata_wei[53]),
    .DI54  (rdata_wei[54]),
    .DI55  (rdata_wei[55]),
    .DI56  (rdata_wei[56]),
    .DI57  (rdata_wei[57]),
    .DI58  (rdata_wei[58]),
    .DI59  (rdata_wei[59]),
    .DI60  (rdata_wei[60]),
    .DI61  (rdata_wei[61]),
    .DI62  (rdata_wei[62]),
    .DI63  (rdata_wei[63]),
    .DI64  (rdata_wei[64]),
    .DI65  (rdata_wei[65]),
    .DI66  (rdata_wei[66]),
    .DI67  (rdata_wei[67]),
    .DI68  (rdata_wei[68]),
    .DI69  (rdata_wei[69]),
    .DI70  (rdata_wei[70]),
    .DI71  (rdata_wei[71]),
    .DI72  (rdata_wei[72]),
    .DI73  (rdata_wei[73]),
    .DI74  (rdata_wei[74]),
    .DI75  (rdata_wei[75]),
    .DI76  (rdata_wei[76]),
    .DI77  (rdata_wei[77]),
    .DI78  (rdata_wei[78]),
    .DI79  (rdata_wei[79]),
    .DI80  (rdata_wei[80]),
    .DI81  (rdata_wei[81]),
    .DI82  (rdata_wei[82]),
    .DI83  (rdata_wei[83]),
    .DI84  (rdata_wei[84]),
    .DI85  (rdata_wei[85]),
    .DI86  (rdata_wei[86]),
    .DI87  (rdata_wei[87]),
    .DI88  (rdata_wei[88]),
    .DI89  (rdata_wei[89]),
    .DI90  (rdata_wei[90]),
    .DI91  (rdata_wei[91]),
    .DI92  (rdata_wei[92]),
    .DI93  (rdata_wei[93]),
    .DI94  (rdata_wei[94]),
    .DI95  (rdata_wei[95]),
    .DI96  (rdata_wei[96]),
    .DI97  (rdata_wei[97]),
    .DI98  (rdata_wei[98]),
    .DI99  (rdata_wei[99]),
    .DI100 (rdata_wei[100]),
    .DI101 (rdata_wei[101]),
    .DI102 (rdata_wei[102]),
    .DI103 (rdata_wei[103]),
    .DI104 (rdata_wei[104]),
    .DI105 (rdata_wei[105]),
    .DI106 (rdata_wei[106]),
    .DI107 (rdata_wei[107]),
    .DI108 (rdata_wei[108]),
    .DI109 (rdata_wei[109]),
    .DI110 (rdata_wei[110]),
    .DI111 (rdata_wei[111]),
    .DI112 (rdata_wei[112]),
    .DI113 (rdata_wei[113]),
    .DI114 (rdata_wei[114]),
    .DI115 (rdata_wei[115]),
    .DI116 (rdata_wei[116]),
    .DI117 (rdata_wei[117]),
    .DI118 (rdata_wei[118]),
    .DI119 (rdata_wei[119]),
    .DI120 (rdata_wei[120]),
    .DI121 (rdata_wei[121]),
    .DI122 (rdata_wei[122]),
    .DI123 (rdata_wei[123]),
    .DI124 (rdata_wei[124]),
    .DI125 (rdata_wei[125]),
    .DI126 (rdata_wei[126]),
    .DI127 (rdata_wei[127]),
    .DO0(DO_W[0]),
    .DO1(DO_W[1]),
    .DO2(DO_W[2]),
    .DO3(DO_W[3]),
    .DO4(DO_W[4]),
    .DO5(DO_W[5]),
    .DO6(DO_W[6]),
    .DO7(DO_W[7]),
    .DO8(DO_W[8]),
    .DO9(DO_W[9]),
    .DO10(DO_W[10]),
    .DO11(DO_W[11]),
    .DO12(DO_W[12]),
    .DO13(DO_W[13]),
    .DO14(DO_W[14]),
    .DO15(DO_W[15]),
    .DO16(DO_W[16]),
    .DO17(DO_W[17]),
    .DO18(DO_W[18]),
    .DO19(DO_W[19]),
    .DO20(DO_W[20]),
    .DO21(DO_W[21]),
    .DO22(DO_W[22]),
    .DO23(DO_W[23]),
    .DO24(DO_W[24]),
    .DO25(DO_W[25]),
    .DO26(DO_W[26]),
    .DO27(DO_W[27]),
    .DO28(DO_W[28]),
    .DO29(DO_W[29]),
    .DO30(DO_W[30]),
    .DO31(DO_W[31]),
    .DO32(DO_W[32]),
    .DO33(DO_W[33]),
    .DO34(DO_W[34]),
    .DO35(DO_W[35]),
    .DO36(DO_W[36]),
    .DO37(DO_W[37]),
    .DO38(DO_W[38]),
    .DO39(DO_W[39]),
    .DO40(DO_W[40]),
    .DO41(DO_W[41]),
    .DO42(DO_W[42]),
    .DO43(DO_W[43]),
    .DO44(DO_W[44]),
    .DO45(DO_W[45]),
    .DO46(DO_W[46]),
    .DO47(DO_W[47]),
    .DO48(DO_W[48]),
    .DO49(DO_W[49]),
    .DO50(DO_W[50]),
    .DO51(DO_W[51]),
    .DO52(DO_W[52]),
    .DO53(DO_W[53]),
    .DO54(DO_W[54]),
    .DO55(DO_W[55]),
    .DO56(DO_W[56]),
    .DO57(DO_W[57]),
    .DO58(DO_W[58]),
    .DO59(DO_W[59]),
    .DO60(DO_W[60]),
    .DO61(DO_W[61]),
    .DO62(DO_W[62]),
    .DO63(DO_W[63]),
    .DO64(DO_W[64]),
    .DO65(DO_W[65]),
    .DO66(DO_W[66]),
    .DO67(DO_W[67]),
    .DO68(DO_W[68]),
    .DO69(DO_W[69]),
    .DO70(DO_W[70]),
    .DO71(DO_W[71]),
    .DO72(DO_W[72]),
    .DO73(DO_W[73]),
    .DO74(DO_W[74]),
    .DO75(DO_W[75]),
    .DO76(DO_W[76]),
    .DO77(DO_W[77]),
    .DO78(DO_W[78]),
    .DO79(DO_W[79]),
    .DO80(DO_W[80]),
    .DO81(DO_W[81]),
    .DO82(DO_W[82]),
    .DO83(DO_W[83]),
    .DO84(DO_W[84]),
    .DO85(DO_W[85]),
    .DO86(DO_W[86]),
    .DO87(DO_W[87]),
    .DO88(DO_W[88]),
    .DO89(DO_W[89]),
    .DO90(DO_W[90]),
    .DO91(DO_W[91]),
    .DO92(DO_W[92]),
    .DO93(DO_W[93]),
    .DO94(DO_W[94]),
    .DO95(DO_W[95]),
    .DO96(DO_W[96]),
    .DO97(DO_W[97]),
    .DO98(DO_W[98]),
    .DO99(DO_W[99]),
    .DO100(DO_W[100]),
    .DO101(DO_W[101]),
    .DO102(DO_W[102]),
    .DO103(DO_W[103]),
    .DO104(DO_W[104]),
    .DO105(DO_W[105]),
    .DO106(DO_W[106]),
    .DO107(DO_W[107]),
    .DO108(DO_W[108]),
    .DO109(DO_W[109]),
    .DO110(DO_W[110]),
    .DO111(DO_W[111]),
    .DO112(DO_W[112]),
    .DO113(DO_W[113]),
    .DO114(DO_W[114]),
    .DO115(DO_W[115]),
    .DO116(DO_W[116]),
    .DO117(DO_W[117]),
    .DO118(DO_W[118]),
    .DO119(DO_W[119]),
    .DO120(DO_W[120]),
    .DO121(DO_W[121]),
    .DO122(DO_W[122]),
    .DO123(DO_W[123]),
    .DO124(DO_W[124]),
    .DO125(DO_W[125]),
    .DO126(DO_W[126]),
    .DO127(DO_W[127]),
    .CK(clk),
    .OE(1'b1),
    .CS(1'b1),
    .WEB(web_wei));

endmodule
