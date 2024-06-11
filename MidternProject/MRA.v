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

module MRA # (
  parameter ID_WIDTH   = 4,
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 128
)(
	// CHIP IO
	clk            	,	
	rst_n          	,	
	in_valid       	,	
	frame_id        ,	
	net_id         	,	  
	loc_x          	,	  
  loc_y         	,
	cost	 	      	,		
	busy         	,

    // AXI4 IO
	     arid_m_inf,
	   araddr_m_inf,
	    arlen_m_inf,
	   arsize_m_inf,
	  arburst_m_inf,
	  arvalid_m_inf,
	  arready_m_inf,
	
	      rid_m_inf,
	    rdata_m_inf,
	    rresp_m_inf,
	    rlast_m_inf,
	   rvalid_m_inf,
	   rready_m_inf,
	
	     awid_m_inf,
	   awaddr_m_inf,
	   awsize_m_inf,
	  awburst_m_inf,
	    awlen_m_inf,
	  awvalid_m_inf,
	  awready_m_inf,
	
	    wdata_m_inf,
	    wlast_m_inf,
	   wvalid_m_inf,
	   wready_m_inf,
	
	      bid_m_inf,
	    bresp_m_inf,
	   bvalid_m_inf,
	   bready_m_inf 
);

// ===============================================================
//  					Input / Output 
// ===============================================================

// << CHIP io port with system >>
input 			  	 clk,rst_n;
input 			   	  in_valid;
input  [4:0] 		  frame_id;
input  [3:0]       	net_id;     
input  [5:0]         loc_x; 
input  [5:0]       	 loc_y; 
output reg [13:0] 	  cost;
output reg            busy;       
  
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
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
output reg [ADDR_WIDTH-1:0]   araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [7:0]              awlen_m_inf;
output reg                  awvalid_m_inf;
input  wire                  awready_m_inf;
output wire [ADDR_WIDTH-1:0]  awaddr_m_inf;
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
reg [DATA_WIDTH-1:0] r_rdata;
reg [DATA_WIDTH-1:0] r_wdata;
reg rvalid_m_inf_d1;


// ===============================================================
// Parameter & Integer Declaration
// ===============================================================

reg flag_map;
reg flag_1;

reg flag_2;
reg flag_3; // use to save input x,y

reg [4:0]  r_frameID;

reg [3:0]  net_ID [0:15];

reg [5:0]  cur_X_d1,cur_Y_d1;  
reg [5:0]  cur_X,cur_Y;
wire [6:0] cur_X_add1, cur_X_minus1;
wire [6:0] cur_Y_add1, cur_Y_minus1;

wire [5:0] add_6bits;
wire add_1bit;
wire [4:0] axis_to_address;
wire [3:0] assign_out;


reg [5:0]  sour_X [0:14];
reg [5:0]  sour_Y [0:14];
reg [5:0]  sink_X [0:14];
reg [5:0]  sink_Y [0:14];

reg in_valid_d1;
reg wready_m_inf_d1;

reg [3:0] cnt_net_num;
reg [9:0] temp_cnt;


reg [9:0] cnt_3;
reg [6:0] cnt_dram;
reg [1:0] cnt_2; //cnt 0~3
wire [127:0] net_line;


reg [3:0] NETnum;
reg [1:0] Map [0:63][0:63];

reg [127:0] temp_reg;
reg [127:0] temp_reg_2;


integer i,j,k,ii,ij,ik,l;
integer X,Y;

// state
parameter S0_IDLE      = 4'd0;
parameter S1_RAVALID   = 4'd1;
parameter S2_read_locMAP_D = 4'd2;
parameter S3_read_weiMAP_D = 4'd3;
parameter S4_setGoal    = 4'd4;
parameter S5_fillMAP = 4'd5;
parameter S6_RETRACE = 4'd6;
parameter S7_JUDGE   = 4'd7;
parameter S8_WAVALID = 4'd8;
parameter S9_write_loc_D = 4'd9;



reg [4:0] c_s, n_s, c_s_d;


//=====================================================================================================================================================================
reg [6:0]  SRAM_loc_add;
reg [6:0]  SRAM_loc_add_d1;
reg [127:0] SRAM_loc_in;
reg [127:0] SRAM_loc_out;
reg SRAM_loc_WEB; 

SUMA180_128X128X1BM1 SRAM_loc(.A0(SRAM_loc_add[0]),.A1(SRAM_loc_add[1]),.A2(SRAM_loc_add[2]),.A3(SRAM_loc_add[3]),.A4(SRAM_loc_add[4]),.A5(SRAM_loc_add[5]),.A6(SRAM_loc_add[6]),
                             .DO0(SRAM_loc_out[0]),.DO1(SRAM_loc_out[1]),.DO2(SRAM_loc_out[2]),.DO3(SRAM_loc_out[3]),.DO4(SRAM_loc_out[4]),.DO5(SRAM_loc_out[5]),.DO6(SRAM_loc_out[6]),.DO7(SRAM_loc_out[7]),
                             .DO8(SRAM_loc_out[8]),.DO9(SRAM_loc_out[9]),.DO10(SRAM_loc_out[10]),.DO11(SRAM_loc_out[11]),.DO12(SRAM_loc_out[12]),.DO13(SRAM_loc_out[13]),.DO14(SRAM_loc_out[14]),.DO15(SRAM_loc_out[15]),
                             .DO16(SRAM_loc_out[16]),.DO17(SRAM_loc_out[17]),.DO18(SRAM_loc_out[18]),.DO19(SRAM_loc_out[19]),.DO20(SRAM_loc_out[20]),.DO21(SRAM_loc_out[21]),.DO22(SRAM_loc_out[22]),.DO23(SRAM_loc_out[23]),
                             .DO24(SRAM_loc_out[24]),.DO25(SRAM_loc_out[25]),.DO26(SRAM_loc_out[26]),.DO27(SRAM_loc_out[27]),.DO28(SRAM_loc_out[28]),.DO29(SRAM_loc_out[29]),.DO30(SRAM_loc_out[30]),.DO31(SRAM_loc_out[31]),
                             .DO32(SRAM_loc_out[32]),.DO33(SRAM_loc_out[33]),.DO34(SRAM_loc_out[34]),.DO35(SRAM_loc_out[35]),.DO36(SRAM_loc_out[36]),.DO37(SRAM_loc_out[37]),.DO38(SRAM_loc_out[38]),.DO39(SRAM_loc_out[39]),
                             .DO40(SRAM_loc_out[40]),.DO41(SRAM_loc_out[41]),.DO42(SRAM_loc_out[42]),.DO43(SRAM_loc_out[43]),.DO44(SRAM_loc_out[44]),.DO45(SRAM_loc_out[45]),.DO46(SRAM_loc_out[46]),.DO47(SRAM_loc_out[47]),
                             .DO48(SRAM_loc_out[48]),.DO49(SRAM_loc_out[49]),.DO50(SRAM_loc_out[50]),.DO51(SRAM_loc_out[51]),.DO52(SRAM_loc_out[52]),.DO53(SRAM_loc_out[53]),.DO54(SRAM_loc_out[54]),.DO55(SRAM_loc_out[55]),
                             .DO56(SRAM_loc_out[56]),.DO57(SRAM_loc_out[57]),.DO58(SRAM_loc_out[58]),.DO59(SRAM_loc_out[59]),.DO60(SRAM_loc_out[60]),.DO61(SRAM_loc_out[61]),.DO62(SRAM_loc_out[62]),.DO63(SRAM_loc_out[63]),
                             .DO64(SRAM_loc_out[64]),.DO65(SRAM_loc_out[65]),.DO66(SRAM_loc_out[66]),.DO67(SRAM_loc_out[67]),.DO68(SRAM_loc_out[68]),.DO69(SRAM_loc_out[69]),.DO70(SRAM_loc_out[70]),.DO71(SRAM_loc_out[71]),
                             .DO72(SRAM_loc_out[72]),.DO73(SRAM_loc_out[73]),.DO74(SRAM_loc_out[74]),.DO75(SRAM_loc_out[75]),.DO76(SRAM_loc_out[76]),.DO77(SRAM_loc_out[77]),.DO78(SRAM_loc_out[78]),.DO79(SRAM_loc_out[79]),
                             .DO80(SRAM_loc_out[80]),.DO81(SRAM_loc_out[81]),.DO82(SRAM_loc_out[82]),.DO83(SRAM_loc_out[83]),.DO84(SRAM_loc_out[84]),.DO85(SRAM_loc_out[85]),.DO86(SRAM_loc_out[86]),.DO87(SRAM_loc_out[87]),
                             .DO88(SRAM_loc_out[88]),.DO89(SRAM_loc_out[89]),.DO90(SRAM_loc_out[90]),.DO91(SRAM_loc_out[91]),.DO92(SRAM_loc_out[92]),.DO93(SRAM_loc_out[93]),.DO94(SRAM_loc_out[94]),.DO95(SRAM_loc_out[95]),
                             .DO96(SRAM_loc_out[96]),.DO97(SRAM_loc_out[97]),.DO98(SRAM_loc_out[98]),.DO99(SRAM_loc_out[99]),.DO100(SRAM_loc_out[100]),.DO101(SRAM_loc_out[101]),.DO102(SRAM_loc_out[102]),.DO103(SRAM_loc_out[103]),
                             .DO104(SRAM_loc_out[104]),.DO105(SRAM_loc_out[105]),.DO106(SRAM_loc_out[106]),.DO107(SRAM_loc_out[107]),.DO108(SRAM_loc_out[108]),.DO109(SRAM_loc_out[109]),.DO110(SRAM_loc_out[110]),.DO111(SRAM_loc_out[111]),
                             .DO112(SRAM_loc_out[112]),.DO113(SRAM_loc_out[113]),.DO114(SRAM_loc_out[114]),.DO115(SRAM_loc_out[115]),.DO116(SRAM_loc_out[116]),.DO117(SRAM_loc_out[117]),.DO118(SRAM_loc_out[118]),.DO119(SRAM_loc_out[119]),
                             .DO120(SRAM_loc_out[120]),.DO121(SRAM_loc_out[121]),.DO122(SRAM_loc_out[122]),.DO123(SRAM_loc_out[123]),.DO124(SRAM_loc_out[124]),.DO125(SRAM_loc_out[125]),.DO126(SRAM_loc_out[126]),.DO127(SRAM_loc_out[127]),
                             .DI0(SRAM_loc_in[0]),.DI1(SRAM_loc_in[1]),.DI2(SRAM_loc_in[2]),.DI3(SRAM_loc_in[3]),.DI4(SRAM_loc_in[4]),.DI5(SRAM_loc_in[5]),.DI6(SRAM_loc_in[6]),.DI7(SRAM_loc_in[7]),
                             .DI8(SRAM_loc_in[8]),.DI9(SRAM_loc_in[9]),.DI10(SRAM_loc_in[10]),.DI11(SRAM_loc_in[11]),.DI12(SRAM_loc_in[12]),.DI13(SRAM_loc_in[13]),.DI14(SRAM_loc_in[14]),.DI15(SRAM_loc_in[15]),
                             .DI16(SRAM_loc_in[16]),.DI17(SRAM_loc_in[17]),.DI18(SRAM_loc_in[18]),.DI19(SRAM_loc_in[19]),.DI20(SRAM_loc_in[20]),.DI21(SRAM_loc_in[21]),.DI22(SRAM_loc_in[22]),.DI23(SRAM_loc_in[23]),
                             .DI24(SRAM_loc_in[24]),.DI25(SRAM_loc_in[25]),.DI26(SRAM_loc_in[26]),.DI27(SRAM_loc_in[27]),.DI28(SRAM_loc_in[28]),.DI29(SRAM_loc_in[29]),.DI30(SRAM_loc_in[30]),.DI31(SRAM_loc_in[31]),
                             .DI32(SRAM_loc_in[32]),.DI33(SRAM_loc_in[33]),.DI34(SRAM_loc_in[34]),.DI35(SRAM_loc_in[35]),.DI36(SRAM_loc_in[36]),.DI37(SRAM_loc_in[37]),.DI38(SRAM_loc_in[38]),.DI39(SRAM_loc_in[39]),
                             .DI40(SRAM_loc_in[40]),.DI41(SRAM_loc_in[41]),.DI42(SRAM_loc_in[42]),.DI43(SRAM_loc_in[43]),.DI44(SRAM_loc_in[44]),.DI45(SRAM_loc_in[45]),.DI46(SRAM_loc_in[46]),.DI47(SRAM_loc_in[47]),
                             .DI48(SRAM_loc_in[48]),.DI49(SRAM_loc_in[49]),.DI50(SRAM_loc_in[50]),.DI51(SRAM_loc_in[51]),.DI52(SRAM_loc_in[52]),.DI53(SRAM_loc_in[53]),.DI54(SRAM_loc_in[54]),.DI55(SRAM_loc_in[55]),
                             .DI56(SRAM_loc_in[56]),.DI57(SRAM_loc_in[57]),.DI58(SRAM_loc_in[58]),.DI59(SRAM_loc_in[59]),.DI60(SRAM_loc_in[60]),.DI61(SRAM_loc_in[61]),.DI62(SRAM_loc_in[62]),.DI63(SRAM_loc_in[63]),
                             .DI64(SRAM_loc_in[64]),.DI65(SRAM_loc_in[65]),.DI66(SRAM_loc_in[66]),.DI67(SRAM_loc_in[67]),.DI68(SRAM_loc_in[68]),.DI69(SRAM_loc_in[69]),.DI70(SRAM_loc_in[70]),.DI71(SRAM_loc_in[71]),
                             .DI72(SRAM_loc_in[72]),.DI73(SRAM_loc_in[73]),.DI74(SRAM_loc_in[74]),.DI75(SRAM_loc_in[75]),.DI76(SRAM_loc_in[76]),.DI77(SRAM_loc_in[77]),.DI78(SRAM_loc_in[78]),.DI79(SRAM_loc_in[79]),
                             .DI80(SRAM_loc_in[80]),.DI81(SRAM_loc_in[81]),.DI82(SRAM_loc_in[82]),.DI83(SRAM_loc_in[83]),.DI84(SRAM_loc_in[84]),.DI85(SRAM_loc_in[85]),.DI86(SRAM_loc_in[86]),.DI87(SRAM_loc_in[87]),
                             .DI88(SRAM_loc_in[88]),.DI89(SRAM_loc_in[89]),.DI90(SRAM_loc_in[90]),.DI91(SRAM_loc_in[91]),.DI92(SRAM_loc_in[92]),.DI93(SRAM_loc_in[93]),.DI94(SRAM_loc_in[94]),.DI95(SRAM_loc_in[95]),
                             .DI96(SRAM_loc_in[96]),.DI97(SRAM_loc_in[97]),.DI98(SRAM_loc_in[98]),.DI99(SRAM_loc_in[99]),.DI100(SRAM_loc_in[100]),.DI101(SRAM_loc_in[101]),.DI102(SRAM_loc_in[102]),.DI103(SRAM_loc_in[103]),
                             .DI104(SRAM_loc_in[104]),.DI105(SRAM_loc_in[105]),.DI106(SRAM_loc_in[106]),.DI107(SRAM_loc_in[107]),.DI108(SRAM_loc_in[108]),.DI109(SRAM_loc_in[109]),.DI110(SRAM_loc_in[110]),.DI111(SRAM_loc_in[111]),
                             .DI112(SRAM_loc_in[112]),.DI113(SRAM_loc_in[113]),.DI114(SRAM_loc_in[114]),.DI115(SRAM_loc_in[115]),.DI116(SRAM_loc_in[116]),.DI117(SRAM_loc_in[117]),.DI118(SRAM_loc_in[118]),.DI119(SRAM_loc_in[119]),
                             .DI120(SRAM_loc_in[120]),.DI121(SRAM_loc_in[121]),.DI122(SRAM_loc_in[122]),.DI123(SRAM_loc_in[123]),.DI124(SRAM_loc_in[124]),.DI125(SRAM_loc_in[125]),.DI126(SRAM_loc_in[126]),.DI127(SRAM_loc_in[127]),
                            .CK(clk),.WEB(SRAM_loc_WEB),.OE(1'b1),.CS(1'b1));

reg  [6:0]  SRAM_wei_add;
reg  [127:0] SRAM_wei_in;
reg  [127:0] SRAM_wei_out;
reg SRAM_wei_WEB; 
  
SUMA180_128X128X1BM1 SRAM_wei(.A0(SRAM_wei_add[0]),.A1(SRAM_wei_add[1]),.A2(SRAM_wei_add[2]),.A3(SRAM_wei_add[3]),.A4(SRAM_wei_add[4]),.A5(SRAM_wei_add[5]),.A6(SRAM_wei_add[6]),
                             .DO0(SRAM_wei_out[0]),.DO1(SRAM_wei_out[1]),.DO2(SRAM_wei_out[2]),.DO3(SRAM_wei_out[3]),.DO4(SRAM_wei_out[4]),.DO5(SRAM_wei_out[5]),.DO6(SRAM_wei_out[6]),.DO7(SRAM_wei_out[7]),
                             .DO8(SRAM_wei_out[8]),.DO9(SRAM_wei_out[9]),.DO10(SRAM_wei_out[10]),.DO11(SRAM_wei_out[11]),.DO12(SRAM_wei_out[12]),.DO13(SRAM_wei_out[13]),.DO14(SRAM_wei_out[14]),.DO15(SRAM_wei_out[15]),
                             .DO16(SRAM_wei_out[16]),.DO17(SRAM_wei_out[17]),.DO18(SRAM_wei_out[18]),.DO19(SRAM_wei_out[19]),.DO20(SRAM_wei_out[20]),.DO21(SRAM_wei_out[21]),.DO22(SRAM_wei_out[22]),.DO23(SRAM_wei_out[23]),
                             .DO24(SRAM_wei_out[24]),.DO25(SRAM_wei_out[25]),.DO26(SRAM_wei_out[26]),.DO27(SRAM_wei_out[27]),.DO28(SRAM_wei_out[28]),.DO29(SRAM_wei_out[29]),.DO30(SRAM_wei_out[30]),.DO31(SRAM_wei_out[31]),
                             .DO32(SRAM_wei_out[32]),.DO33(SRAM_wei_out[33]),.DO34(SRAM_wei_out[34]),.DO35(SRAM_wei_out[35]),.DO36(SRAM_wei_out[36]),.DO37(SRAM_wei_out[37]),.DO38(SRAM_wei_out[38]),.DO39(SRAM_wei_out[39]),
                             .DO40(SRAM_wei_out[40]),.DO41(SRAM_wei_out[41]),.DO42(SRAM_wei_out[42]),.DO43(SRAM_wei_out[43]),.DO44(SRAM_wei_out[44]),.DO45(SRAM_wei_out[45]),.DO46(SRAM_wei_out[46]),.DO47(SRAM_wei_out[47]),
                             .DO48(SRAM_wei_out[48]),.DO49(SRAM_wei_out[49]),.DO50(SRAM_wei_out[50]),.DO51(SRAM_wei_out[51]),.DO52(SRAM_wei_out[52]),.DO53(SRAM_wei_out[53]),.DO54(SRAM_wei_out[54]),.DO55(SRAM_wei_out[55]),
                             .DO56(SRAM_wei_out[56]),.DO57(SRAM_wei_out[57]),.DO58(SRAM_wei_out[58]),.DO59(SRAM_wei_out[59]),.DO60(SRAM_wei_out[60]),.DO61(SRAM_wei_out[61]),.DO62(SRAM_wei_out[62]),.DO63(SRAM_wei_out[63]),
                             .DO64(SRAM_wei_out[64]),.DO65(SRAM_wei_out[65]),.DO66(SRAM_wei_out[66]),.DO67(SRAM_wei_out[67]),.DO68(SRAM_wei_out[68]),.DO69(SRAM_wei_out[69]),.DO70(SRAM_wei_out[70]),.DO71(SRAM_wei_out[71]),
                             .DO72(SRAM_wei_out[72]),.DO73(SRAM_wei_out[73]),.DO74(SRAM_wei_out[74]),.DO75(SRAM_wei_out[75]),.DO76(SRAM_wei_out[76]),.DO77(SRAM_wei_out[77]),.DO78(SRAM_wei_out[78]),.DO79(SRAM_wei_out[79]),
                             .DO80(SRAM_wei_out[80]),.DO81(SRAM_wei_out[81]),.DO82(SRAM_wei_out[82]),.DO83(SRAM_wei_out[83]),.DO84(SRAM_wei_out[84]),.DO85(SRAM_wei_out[85]),.DO86(SRAM_wei_out[86]),.DO87(SRAM_wei_out[87]),
                             .DO88(SRAM_wei_out[88]),.DO89(SRAM_wei_out[89]),.DO90(SRAM_wei_out[90]),.DO91(SRAM_wei_out[91]),.DO92(SRAM_wei_out[92]),.DO93(SRAM_wei_out[93]),.DO94(SRAM_wei_out[94]),.DO95(SRAM_wei_out[95]),
                             .DO96(SRAM_wei_out[96]),.DO97(SRAM_wei_out[97]),.DO98(SRAM_wei_out[98]),.DO99(SRAM_wei_out[99]),.DO100(SRAM_wei_out[100]),.DO101(SRAM_wei_out[101]),.DO102(SRAM_wei_out[102]),.DO103(SRAM_wei_out[103]),
                             .DO104(SRAM_wei_out[104]),.DO105(SRAM_wei_out[105]),.DO106(SRAM_wei_out[106]),.DO107(SRAM_wei_out[107]),.DO108(SRAM_wei_out[108]),.DO109(SRAM_wei_out[109]),.DO110(SRAM_wei_out[110]),.DO111(SRAM_wei_out[111]),
                             .DO112(SRAM_wei_out[112]),.DO113(SRAM_wei_out[113]),.DO114(SRAM_wei_out[114]),.DO115(SRAM_wei_out[115]),.DO116(SRAM_wei_out[116]),.DO117(SRAM_wei_out[117]),.DO118(SRAM_wei_out[118]),.DO119(SRAM_wei_out[119]),
                             .DO120(SRAM_wei_out[120]),.DO121(SRAM_wei_out[121]),.DO122(SRAM_wei_out[122]),.DO123(SRAM_wei_out[123]),.DO124(SRAM_wei_out[124]),.DO125(SRAM_wei_out[125]),.DO126(SRAM_wei_out[126]),.DO127(SRAM_wei_out[127]),
                             .DI0(SRAM_wei_in[0]),.DI1(SRAM_wei_in[1]),.DI2(SRAM_wei_in[2]),.DI3(SRAM_wei_in[3]),.DI4(SRAM_wei_in[4]),.DI5(SRAM_wei_in[5]),.DI6(SRAM_wei_in[6]),.DI7(SRAM_wei_in[7]),
                             .DI8(SRAM_wei_in[8]),.DI9(SRAM_wei_in[9]),.DI10(SRAM_wei_in[10]),.DI11(SRAM_wei_in[11]),.DI12(SRAM_wei_in[12]),.DI13(SRAM_wei_in[13]),.DI14(SRAM_wei_in[14]),.DI15(SRAM_wei_in[15]),
                             .DI16(SRAM_wei_in[16]),.DI17(SRAM_wei_in[17]),.DI18(SRAM_wei_in[18]),.DI19(SRAM_wei_in[19]),.DI20(SRAM_wei_in[20]),.DI21(SRAM_wei_in[21]),.DI22(SRAM_wei_in[22]),.DI23(SRAM_wei_in[23]),
                             .DI24(SRAM_wei_in[24]),.DI25(SRAM_wei_in[25]),.DI26(SRAM_wei_in[26]),.DI27(SRAM_wei_in[27]),.DI28(SRAM_wei_in[28]),.DI29(SRAM_wei_in[29]),.DI30(SRAM_wei_in[30]),.DI31(SRAM_wei_in[31]),
                             .DI32(SRAM_wei_in[32]),.DI33(SRAM_wei_in[33]),.DI34(SRAM_wei_in[34]),.DI35(SRAM_wei_in[35]),.DI36(SRAM_wei_in[36]),.DI37(SRAM_wei_in[37]),.DI38(SRAM_wei_in[38]),.DI39(SRAM_wei_in[39]),
                             .DI40(SRAM_wei_in[40]),.DI41(SRAM_wei_in[41]),.DI42(SRAM_wei_in[42]),.DI43(SRAM_wei_in[43]),.DI44(SRAM_wei_in[44]),.DI45(SRAM_wei_in[45]),.DI46(SRAM_wei_in[46]),.DI47(SRAM_wei_in[47]),
                             .DI48(SRAM_wei_in[48]),.DI49(SRAM_wei_in[49]),.DI50(SRAM_wei_in[50]),.DI51(SRAM_wei_in[51]),.DI52(SRAM_wei_in[52]),.DI53(SRAM_wei_in[53]),.DI54(SRAM_wei_in[54]),.DI55(SRAM_wei_in[55]),
                             .DI56(SRAM_wei_in[56]),.DI57(SRAM_wei_in[57]),.DI58(SRAM_wei_in[58]),.DI59(SRAM_wei_in[59]),.DI60(SRAM_wei_in[60]),.DI61(SRAM_wei_in[61]),.DI62(SRAM_wei_in[62]),.DI63(SRAM_wei_in[63]),
                             .DI64(SRAM_wei_in[64]),.DI65(SRAM_wei_in[65]),.DI66(SRAM_wei_in[66]),.DI67(SRAM_wei_in[67]),.DI68(SRAM_wei_in[68]),.DI69(SRAM_wei_in[69]),.DI70(SRAM_wei_in[70]),.DI71(SRAM_wei_in[71]),
                             .DI72(SRAM_wei_in[72]),.DI73(SRAM_wei_in[73]),.DI74(SRAM_wei_in[74]),.DI75(SRAM_wei_in[75]),.DI76(SRAM_wei_in[76]),.DI77(SRAM_wei_in[77]),.DI78(SRAM_wei_in[78]),.DI79(SRAM_wei_in[79]),
                             .DI80(SRAM_wei_in[80]),.DI81(SRAM_wei_in[81]),.DI82(SRAM_wei_in[82]),.DI83(SRAM_wei_in[83]),.DI84(SRAM_wei_in[84]),.DI85(SRAM_wei_in[85]),.DI86(SRAM_wei_in[86]),.DI87(SRAM_wei_in[87]),
                             .DI88(SRAM_wei_in[88]),.DI89(SRAM_wei_in[89]),.DI90(SRAM_wei_in[90]),.DI91(SRAM_wei_in[91]),.DI92(SRAM_wei_in[92]),.DI93(SRAM_wei_in[93]),.DI94(SRAM_wei_in[94]),.DI95(SRAM_wei_in[95]),
                             .DI96(SRAM_wei_in[96]),.DI97(SRAM_wei_in[97]),.DI98(SRAM_wei_in[98]),.DI99(SRAM_wei_in[99]),.DI100(SRAM_wei_in[100]),.DI101(SRAM_wei_in[101]),.DI102(SRAM_wei_in[102]),.DI103(SRAM_wei_in[103]),
                             .DI104(SRAM_wei_in[104]),.DI105(SRAM_wei_in[105]),.DI106(SRAM_wei_in[106]),.DI107(SRAM_wei_in[107]),.DI108(SRAM_wei_in[108]),.DI109(SRAM_wei_in[109]),.DI110(SRAM_wei_in[110]),.DI111(SRAM_wei_in[111]),
                             .DI112(SRAM_wei_in[112]),.DI113(SRAM_wei_in[113]),.DI114(SRAM_wei_in[114]),.DI115(SRAM_wei_in[115]),.DI116(SRAM_wei_in[116]),.DI117(SRAM_wei_in[117]),.DI118(SRAM_wei_in[118]),.DI119(SRAM_wei_in[119]),
                             .DI120(SRAM_wei_in[120]),.DI121(SRAM_wei_in[121]),.DI122(SRAM_wei_in[122]),.DI123(SRAM_wei_in[123]),.DI124(SRAM_wei_in[124]),.DI125(SRAM_wei_in[125]),.DI126(SRAM_wei_in[126]),.DI127(SRAM_wei_in[127]),
                            .CK(clk),.WEB(SRAM_wei_WEB),.OE(1'b1),.CS(1'b1));

//================================================================
//                FSM
//================================================================
always @(posedge clk or negedge rst_n)
begin
    if (!rst_n)
        c_s <= S0_IDLE ;
    else
        c_s <= n_s ;
end
always @(*)
begin
    n_s = c_s ;
    case(c_s)
      S0_IDLE:
        if (in_valid)    n_s = S1_RAVALID;
        else             n_s = S0_IDLE ;
      S1_RAVALID:
        if (arready_m_inf) begin
          if (flag_map)  n_s = S3_read_weiMAP_D;
          else           n_s = S2_read_locMAP_D;
        end
        else             n_s = S1_RAVALID ;
      S2_read_locMAP_D:
        if (rlast_m_inf)  n_s = S1_RAVALID;
        else              n_s = S2_read_locMAP_D ;
		  S3_read_weiMAP_D:
        if (rlast_m_inf)  n_s = S4_setGoal;
        else              n_s = S3_read_weiMAP_D ;	
      S4_setGoal:
        // if(cnt_net_num_ID == NETnum)    n_s = S9_write_loc_D ; 
        // else                          n_s = S5_fillMAP;     	
                         n_s = S5_fillMAP;     	
		  S5_fillMAP:
        // if (flag_1)
        if(Map[sink_Y[cnt_net_num]][sink_X[cnt_net_num]]!=0)
            n_s = S6_RETRACE;
        else
            n_s = S5_fillMAP ;
		  S6_RETRACE:
        // if (cur_X_d1==sour_X[cnt_net_num] && cur_Y_d1==sour_Y[cnt_net_num])
        if (cur_X==sour_X[cnt_net_num] && cur_Y==sour_Y[cnt_net_num] )
            n_s = S7_JUDGE;
        else
            n_s = S6_RETRACE ;	
      S7_JUDGE:
        if((cnt_net_num+1)==NETnum) 
          n_s = S8_WAVALID;
        else 
          n_s = S4_setGoal;
      S8_WAVALID:
            if (awready_m_inf)
              n_s = S9_write_loc_D;
            else
              n_s = S8_WAVALID;											
		  S9_write_loc_D :
          if (bvalid_m_inf) n_s = S0_IDLE;
          else              n_s = S9_write_loc_D ;
        default:
            n_s = S0_IDLE;
    endcase
end
// ===============================================================
//      cnt & DELAY & FLAG
// ===============================================================
always@(posedge clk or negedge rst_n)
begin
  if (!rst_n) in_valid_d1 <= 0;
  else in_valid_d1 <= in_valid;
end

always@(posedge clk or negedge rst_n)
begin
  if (!rst_n) wready_m_inf_d1 <= 0;
  else wready_m_inf_d1 <= wready_m_inf;
end
always@(posedge clk or negedge rst_n)
begin
  if (!rst_n) cnt_net_num <= 0;
  else if(c_s==S7_JUDGE) begin
    cnt_net_num <= cnt_net_num + 1;
  end
  else if(c_s==S0_IDLE) cnt_net_num <= 0;
end


// ===============================================================
//  				      cnt_2
// ===============================================================
  always @ (posedge clk or negedge rst_n) begin 
  	if (!rst_n) cnt_2 <= 0 ;
  	else begin
  		if (c_s == S0_IDLE)                                                    cnt_2 <= 0 ;
  		else if(c_s == S4_setGoal || (c_s == S5_fillMAP && n_s == S5_fillMAP)) cnt_2 <= cnt_2 + 1 ;
  		else if(c_s == S5_fillMAP && n_s == S6_RETRACE )                        cnt_2 <= cnt_2 - 2 ; 
  		else if(c_s == S6_RETRACE && cnt_3==2)                                  cnt_2 <= cnt_2 - 1 ; 
      else if(c_s ==S7_JUDGE) cnt_2 <= 0;
  		// else if(c_s == S8_WAVALID && n_s == S4_setGoal)                    cnt_2 <= 0 ;      
  		else cnt_2 <= cnt_2 ;
  	end
  end
  always @ (posedge clk or negedge rst_n) begin 
  	if (!rst_n) cnt_3 <= 0 ;  
    else if(in_valid) cnt_3 <= cnt_3 + 1;
    else if(c_s==S0_IDLE) cnt_3 <= 0;
    else if(c_s==S5_fillMAP) cnt_3 <= 0;
    else if(c_s==S6_RETRACE && SRAM_loc_WEB) begin
      if(cnt_3==2) cnt_3 <= 0;
      else cnt_3 <= cnt_3 + 1;
    end
    else cnt_3 <= cnt_3;
  end

// ------- cnt_dram -----------------------
  always @(posedge clk or negedge rst_n) 
  begin
    if(!rst_n) cnt_dram <= 0;
    else begin
      if(rvalid_m_inf || wready_m_inf) cnt_dram <= cnt_dram + 1;
      else cnt_dram <= 0;
    end
  end
  always @(posedge clk or negedge rst_n) 
  begin
    if(!rst_n) temp_cnt <= 0;
    else begin
      if(c_s==S9_write_loc_D) temp_cnt <= temp_cnt + 1;
      else temp_cnt <= 0;
    end
  end
// --------- flag --------------------------
always @(posedge clk or negedge rst_n) 
begin
  if(!rst_n) flag_3 <= 0;
  else if(in_valid) flag_3 <= ~flag_3;
  else flag_3 <= flag_3;
end
always @(posedge clk or negedge rst_n) 
begin
  if(!rst_n) flag_map<=0;
  else if(rlast_m_inf) flag_map <= 1;
  else if(c_s==S0_IDLE) flag_map <= 0;
  else   flag_map<=flag_map;
end

always @(posedge clk or negedge rst_n) 
begin
  if(!rst_n) flag_1 <= 0;
  else if(rlast_m_inf) flag_1 <= 0;
  else if(c_s==S4_setGoal) flag_1 <= 0; 
  else if(c_s==S5_fillMAP)begin
    // if(Map[sink_Y[cnt_net_num]+1][sink_X[cnt_net_num]]!=0 && Map[sink_Y[cnt_net_num]-1][sink_X[cnt_net_num]]!=0 && Map[sink_Y[cnt_net_num]][sink_X[cnt_net_num]+1]!=0 && Map[sink_Y[cnt_net_num]][sink_X[cnt_net_num]-1]!=0) flag_1 <= 1;
    if(n_s==S6_RETRACE) flag_1 <= 1;
    else flag_1 <= 0;  
  end
  else if(c_s==S6_RETRACE) begin
    if((cur_X!=cur_X_d1)||(cur_Y!=cur_Y_d1)) flag_1 <= 0;
  end
  else if(c_s==S0_IDLE) flag_1 <= 0;
  else   flag_1 <= flag_1;
end


// ===============================================================
//      INPUT
// ===============================================================
always@(posedge clk or negedge rst_n) // reg frame_id
  begin
    if(!rst_n) r_frameID <= 0;
    else if (in_valid && !in_valid_d1) r_frameID <= frame_id;
    else if (c_s == S0_IDLE) r_frameID <= 0;
  end
always@(posedge clk or negedge rst_n) // total number of nets
  begin
    if(!rst_n) NETnum <= 0;
    else if (in_valid && cnt_3[0]==1) NETnum <= NETnum+1;
    else if (c_s == S0_IDLE) NETnum <= 0;
  end

always@(posedge clk or negedge rst_n) // reg frame_id
  begin
    if(!rst_n) begin
      for(i=0;i<16;i=i+1) begin
        net_ID[i] <= 0;
      end
    end
    else begin
      if(in_valid && cnt_3[0]==0) net_ID[cnt_3[4:1]] <= net_id;
    end
  end
//--------------source & sink ------------------------------
  //------- SOURCE ------------------------------------
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) begin
          for (i=0; i<15; i=i+1) begin
            sour_X[i] <= 0;  
          end
        end
        else if(in_valid && !flag_3) sour_X[NETnum] <= loc_x;
        else begin
          for (i=0; i<15; i=i+1) begin
            sour_X[i] <= sour_X[i];  
          end
        end 
      end
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) begin
          for (i=0; i<15; i=i+1) begin
            sour_Y[i] <= 0;  
          end
        end
        else if(in_valid && !flag_3) sour_Y[NETnum] <= loc_y;
        else begin
          for (i=0; i<15; i=i+1) begin
            sour_Y[i] <= sour_Y[i];  
          end
        end 
      end
  //-------- SINK --------------------------------------
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) begin
          for (i=0; i<15; i=i+1) begin
            sink_X[i] <= 0;  
          end
        end
        else if(in_valid && flag_3) sink_X[NETnum] <= loc_x;
        else begin
          for (i=0; i<15; i=i+1) begin
            sink_X[i] <= sink_X[i];  
          end
        end 
      end
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) begin
          for (i=0; i<15; i=i+1) begin
            sink_Y[i] <= 0;  
          end
        end
        else if(in_valid && flag_3) sink_Y[NETnum] <= loc_y;
        else begin
          for (i=0; i<15; i=i+1) begin
            sink_Y[i] <= sink_Y[i];  
          end
        end 
      end
  

    
  
// -----------------------------------------------------------------
//    MMM     MMMM       AAAAAA        PPPPPPPPP 
//    MMMMM  MMMMM      AAA  AAA      PPP    PPP
//    MM  MMMM  MM     AAA    AAA     PPPPPPPPP
//    MM   MM   MM    AAAAAAAAAAAA    PPP
//    MM        MM   AAA        AAA   PPP  
// -----------------------------------------------------------------
always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n)begin
  	    for(i=0; i<64; i=i+1)begin
  	      for(j=0; j<64; j=j+1) begin
  	        Map[i][j] <= 0;
  	      end
  	    end
      end
    else begin
      if(c_s==S2_read_locMAP_D && rvalid_m_inf) 
        begin   
           for (k = 0; k < 32; k = k + 1)  
              begin
                Map[cnt_dram>>1][((cnt_dram[0])<<5)+k] <= {1'b0,(|rdata_m_inf[k<<2+:4])}; // 0: road, 1: object or net, 2,3: wave propagation
                // Map[cnt_dram/2][(cnt_dram%2)*32+k] <= {1'b0,(|rdata_m_inf[k*4+:4])}; // 0: road, 1: object or net, 2,3: wave propagation
              end
        end
      else if(c_s==S4_setGoal) 
        begin
          Map[sour_Y[cnt_net_num]][sour_X[cnt_net_num]] <= 2; // propagate from source
          Map[sink_Y[cnt_net_num]][sink_X[cnt_net_num]] <= 0; // sink is the goal of propagation
        for (X = 0 ; X < 64 ; X = X + 1) begin 
				  for (Y = 0 ; Y < 64 ; Y = Y + 1) begin 
				  	if (Map[Y][X][1]) begin 
				  		Map[Y][X] <= 0 ;  
				  	end
				end
			end
        end
      else if(c_s==S5_fillMAP) 
        begin
			  // middle
			    for (X = 1 ; X < 63 ; X = X + 1) begin  
			    	for (Y = 1 ; Y < 63 ; Y = Y + 1) begin 
			    		if (Map[Y][X] == 0 && (Map[Y-1][X][1] | Map[Y+1][X][1] | Map[Y][X-1][1] | Map[Y][X+1][1])) Map[Y][X] <= {1'b1, cnt_2[1]};
			    	end
			    end
			  // boundary
			    for (Y = 1 ; Y < 63 ; Y = Y + 1) begin 
			    	if (Map[Y][0]  == 0 && (Map[Y+1][0][1]  | Map[Y-1][0][1]  | Map[Y][1][1]))  Map[Y][0]  <= {1'b1, cnt_2[1]} ;
			    	if (Map[Y][63] == 0 && (Map[Y+1][63][1] | Map[Y-1][63][1] | Map[Y][62][1])) Map[Y][63] <= {1'b1, cnt_2[1]} ;
			    end
			    for (X = 1 ; X < 63 ; X = X + 1) begin 
			    	if (Map[0][X]  == 0 && (Map[0][X+1][1]  | Map[0][X-1][1]  | Map[1][X][1]))  Map[0][X]  <= {1'b1, cnt_2[1]} ;
			    	if (Map[63][X] == 0 && (Map[63][X+1][1] | Map[63][X-1][1] | Map[62][X][1])) Map[63][X] <= {1'b1, cnt_2[1]} ;
			    end
        // corner
			    if (Map[0][0] == 0   && (Map[0][1][1]   | Map[1][0][1]))   Map[0][0]   <= {1'b1, cnt_2[1]} ;
			    if (Map[63][63] == 0 && (Map[63][62][1] | Map[62][63][1])) Map[63][63] <= {1'b1, cnt_2[1]} ;
			    if (Map[0][63]  == 0 && (Map[1][63][1]  | Map[0][62][1]))  Map[0][63]  <= {1'b1, cnt_2[1]} ;
			    if (Map[63][0]  == 0 && (Map[63][1][1]  | Map[62][0][1]))  Map[63][0]  <= {1'b1, cnt_2[1]} ;           
        end
      else if(c_s==S6_RETRACE || c_s==S7_JUDGE)
        begin
          Map[cur_Y_d1][cur_X_d1] <= 1; // make net
        end
      else begin
        for(i=0; i<64; i=i+1)begin
  	      for(j=0; j<64; j=j+1) begin
  	        Map[i][j] <=  Map[i][j];
  	      end
  	    end
      end  
    end 
  end


// -----------------------------------------------------------------
//   RRRRRRRRR   EEEEEEEEE  TTTTTTTTT  RRRRRRRRR      AAAA     
//   RR     RRR  EE            TT      RR     RRR    AA  AA    
//   RRRRRRRRR   EEEEEEEE      TT      RRRRRRRRR    AA    AA   
//   RR    RR    EE            TT      RR    RR    AAAAAAAAAA  
//   RR     RRR  EEEEEEEEE     TT      RR     RRR AA        AA 
// -----------------------------------------------------------------
  // convience to write
  assign cur_X_add1 = cur_X +1; 
  assign cur_X_minus1 = cur_X -1;
  assign cur_Y_add1 = cur_Y +1;
  assign cur_Y_minus1 = cur_Y -1;

// Current coordinate
  always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
      cur_X <= 0;
      cur_Y <= 0;
    end
    else begin
      if(c_s == S5_fillMAP)begin
        cur_X <= sink_X[cnt_net_num];
        cur_Y <= sink_Y[cnt_net_num];
      end
      else if(c_s == S6_RETRACE && cnt_3==2) begin 
        if((~cur_Y_add1[6]) && Map[cur_Y_add1][cur_X]=={1'b1,cnt_2[1]}) begin
          cur_X <= cur_X;
          cur_Y <= cur_Y_add1;
        end
        else if((~cur_Y_minus1[6]) && Map[cur_Y_minus1][cur_X]=={1'b1,cnt_2[1]})begin
          cur_X <= cur_X;
          cur_Y <= cur_Y_minus1;          
        end
        else if((~cur_X_add1[6]) && Map[cur_Y][cur_X_add1]=={1'b1,cnt_2[1]})begin
          cur_X <= cur_X_add1;
          cur_Y <= cur_Y;          
        end
        else if((~cur_X_minus1[6]) && Map[cur_Y][cur_X_minus1]=={1'b1,cnt_2[1]})begin
          cur_X <= cur_X_minus1;
          cur_Y <= cur_Y;          
        end
        else begin
          cur_X <= cur_X;
          cur_Y <= cur_Y;
        end                
      end
    end
  end
// Last coordinate
  always@(posedge clk or negedge rst_n)begin
    if(!rst_n) cur_X_d1 <= 0;
    else begin
      if(c_s==S5_fillMAP) cur_X_d1 <= cur_X;
      else if(c_s==S6_RETRACE) cur_X_d1 <= cur_X;
    end
  end
  always@(posedge clk or negedge rst_n)begin
    if(!rst_n) cur_Y_d1 <= 0;
    else begin
      if(c_s==S5_fillMAP) cur_Y_d1 <= cur_Y;
      else if(c_s==S6_RETRACE) cur_Y_d1 <= cur_Y;
    end
  end    

// -----------------------------------------------------------------
//  DDDDDDDD    RRRRRRRRR      AAAA     MMM     MMMM 
//  DD    DDD   RR     RRR    AA  AA    MMMMM  MMMMM 
//  DD     DDD  RRRRRRRRR    AA    AA   MM  MMMM  MM 
//  DD    DDD   RR    RR    AAAAAAAAAA  MM   MM   MM 
//  DDDDDDD     RR     RRR AA        AA MM        MM 
// -----------------------------------------------------------------
// <<<<< AXI READ >>>>>
// ------------------------
  // --- 	axi read address channel  ---------------------
    assign arid_m_inf = 'b0;
    assign arburst_m_inf = 2'b01; ///limited to be 2'b01
    assign arsize_m_inf = 3'b100; //limited to be 3'b100 which is 16 Bytes
    assign arlen_m_inf = 'd127; // (64*64)/16 one address can store 16 grids
    
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n) arvalid_m_inf <= 0;
    	else if (arready_m_inf)   arvalid_m_inf <= 0;	
    	else if (c_s==S1_RAVALID) arvalid_m_inf <= 1;
    end
    
    always@(posedge clk or negedge rst_n)
    begin
    	if(!rst_n) araddr_m_inf <= 0;
    	else if(c_s==S1_RAVALID) begin
        if(flag_map)       araddr_m_inf <= 32'h00020000 + r_frameID*'h800;
        else               araddr_m_inf <= 32'h00010000 + r_frameID*'h800; 
      end
    end
    
  // --- 	axi read data channel  ---------------------
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) rready_m_inf <= 0;
        else if(arready_m_inf) rready_m_inf <= 1;
        else if(rlast_m_inf)   rready_m_inf <= 0;
      end

    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) rvalid_m_inf_d1 <= 0;
        else rvalid_m_inf_d1 <= rvalid_m_inf;
      end  
    always@(posedge clk or negedge rst_n)
      begin
        if(!rst_n) r_rdata <= 0;
        else r_rdata <= rdata_m_inf;
      end 
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
assign awid_m_inf    = 4'b0;
assign awburst_m_inf = 2'b01;
assign awsize_m_inf  = 3'b100;
assign awlen_m_inf   = 127; 

// (1) 	axi write address channel
 assign   awaddr_m_inf = 32'h00010000 + r_frameID*'h800;
 always@(posedge clk or negedge rst_n) begin
   if(!rst_n) awvalid_m_inf <= 0;
   else begin
    if (awready_m_inf)    awvalid_m_inf<=0;
    else if(c_s==S8_WAVALID) awvalid_m_inf <= 1;
    else awvalid_m_inf <= 0;
   end
 end 
// (2)	axi write data channel 
assign wlast_m_inf = (cnt_dram==127)? 1:0;
assign wvalid_m_inf = 1;
assign wdata_m_inf = r_wdata;

always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n) r_wdata <= 0;
    else if(wready_m_inf && !wready_m_inf_d1) r_wdata <= temp_reg;
    else if(wready_m_inf) r_wdata <= SRAM_loc_out;
    else r_wdata <= temp_reg_2;
  end
always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n) temp_reg <= 0;
    else if(c_s==S9_write_loc_D && SRAM_loc_add_d1==1) temp_reg <= SRAM_loc_out;
  end
always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n) temp_reg_2 <= 0;
    else if(c_s==S8_WAVALID) temp_reg_2 <= SRAM_loc_out;
  end  
always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n) SRAM_loc_add_d1 <= 0;
    else SRAM_loc_add_d1 <= SRAM_loc_add;
  end
  

// (3)	axi write response channel 
assign  bid_m_inf = 'b0;
assign bready_m_inf = 1;

// =========================================================
//         SSSSSSSS  RRRRRRRRR      AAAA     MMM     MMMM           
//         SS        RR     RRR    AA  AA    MMMMM  MMMMM        
//         SSSSSSSS  RRRRRRRRR    AA    AA   MM  MMMM  MM        
//               SS  RR    RR    AAAAAAAAAA  MM   MM   MM      
//         SSSSSSSS  RR     RRR AA        AA MM        MM 
//        
// ======== SRAM_loc ===================================
 // ---------- A --------------------------
  always@(posedge clk or negedge rst_n) //SRAM_loc_WEB
    begin
      if(!rst_n) SRAM_loc_WEB <= 0;
      else if(c_s==S4_setGoal) SRAM_loc_WEB <= 1;
      else if(rvalid_m_inf_d1) begin
        if(c_s==S3_read_weiMAP_D)  SRAM_loc_WEB <= 1;
        else                    SRAM_loc_WEB <= 0;
      end
      else if(c_s==S6_RETRACE)begin
        if(cnt_3==2) SRAM_loc_WEB <= 0;
        else  SRAM_loc_WEB <= 1;
      end
      else SRAM_loc_WEB <= 1;
    end
  always@(posedge clk or negedge rst_n) //SRAM128_add
    begin // SRAM_loc_add [6:0]
      if(!rst_n) SRAM_loc_add <= 0;
      else if(c_s==S5_fillMAP) SRAM_loc_add <= {add_6bits,add_1bit};
        
      else if(wready_m_inf) SRAM_loc_add <= SRAM_loc_add+1;

      else if(c_s==S6_RETRACE) SRAM_loc_add <= {add_6bits,add_1bit};
      else if(c_s==S2_read_locMAP_D && !SRAM_loc_WEB) SRAM_loc_add <= SRAM_loc_add+1;
      else if(c_s==S1_RAVALID && !SRAM_loc_WEB) SRAM_loc_add <= SRAM_loc_add+1;
      else if(c_s==S9_write_loc_D) SRAM_loc_add <= 2;
      else if(n_s==S9_write_loc_D) SRAM_loc_add <= 1;
      else if(c_s==S8_WAVALID) SRAM_loc_add <= 0;      
      else if(c_s==S0_IDLE) SRAM_loc_add <= 0;
      else SRAM_loc_add <= SRAM_loc_add;
    end


  always@(posedge clk or negedge rst_n) //SRAM128_in
    begin
      if(!rst_n)  SRAM_loc_in <= 0;
      else if(rvalid_m_inf_d1) SRAM_loc_in <= r_rdata;
      else if(c_s==S6_RETRACE)begin
        if(flag_1) SRAM_loc_in <= SRAM_loc_out;
        else       SRAM_loc_in <= SRAM_loc_out + net_line;
      end
      else SRAM_loc_in <= SRAM_loc_in;
    end


// ======== SRAM_wei ===================================
  assign add_1bit  = (cur_X>31)? 1 : 0;
  assign add_6bits = cur_Y;
 // ---------- A --------------------------
  always@(posedge clk or negedge rst_n) //SRAM_wei_WEB
    begin
      if(!rst_n) SRAM_wei_WEB <= 0;
      else if(rvalid_m_inf_d1) begin
        if(c_s==S3_read_weiMAP_D || c_s==S4_setGoal) SRAM_wei_WEB <= 0;
        else         SRAM_wei_WEB <= 1;
      end
      else SRAM_wei_WEB <= 1;
    end
  always@(posedge clk or negedge rst_n) //SRAM_wei_add [6:0]
    begin 
      if(!rst_n)  SRAM_wei_add <= 0;
      else begin
        if(!SRAM_wei_WEB)   SRAM_wei_add <= SRAM_wei_add+1;      
        else if(c_s==S6_RETRACE) SRAM_wei_add <= {add_6bits,add_1bit};
        else SRAM_wei_add <= 0;
      end
    end
  always@(posedge clk or negedge rst_n) //SRAM128_in
    begin
      if(!rst_n)  SRAM_wei_in <= 0;
      else if(rvalid_m_inf_d1) begin
        SRAM_wei_in <= r_rdata;
      end
    end

// ===============================================================
//      OUTPUT
// ===============================================================
assign axis_to_address = (cur_X>31)? cur_X-'d32 : cur_X;
assign assign_out = SRAM_wei_out >> (axis_to_address*4);
assign net_line = net_ID[cnt_net_num] << (axis_to_address*4); 
  

always@(posedge clk or negedge rst_n)
begin
  if(!rst_n)  flag_2 <= 0;
  else if(c_s==S6_RETRACE && (cur_X_d1!=sink_X[cnt_net_num] || cur_Y_d1!=sink_Y[cnt_net_num])) flag_2 <= 1;
  else if(c_s==S0_IDLE) flag_2 <= 0;
end
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)  busy <= 0;
    else if(n_s == S0_IDLE || in_valid)  busy <= 0;
    else    busy <= 1;
end

always @(posedge clk or negedge rst_n) 
  begin
    if(!rst_n) cost <= 0;
    else if(c_s==S6_RETRACE && cnt_3==2 && !flag_1) begin
      // if(cur_X==sour_X && cur_Y==sour_Y) cost <= cost;
      // if(cur_X_d1!=sink_X[cnt_net_num] || cur_Y_d1!=sink_Y[cnt_net_num]) cost <= cost + SRAM_wei_out[axis_to_address*4 +: 4];
      if(flag_2) cost <= cost + assign_out;
    end
    else if(c_s==S0_IDLE) cost <= 0;
    else cost <= cost;
  end


endmodule
