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
reg flag_1_d;
reg flag_2;
reg flag_3; // use to save input x,y

reg [4:0]  r_frameID;
reg [3:0]  exe_net; // the net is executing
reg [3:0]  net_ID [0:15];

reg [5:0]  cur_X_d2,cur_Y_d2;  
reg [5:0]  cur_X_d1,cur_Y_d1;  
reg [5:0]  cur_X,cur_Y;
wire [6:0] cur_X_add1, cur_X_minus1;
wire [6:0] cur_Y_add1, cur_Y_minus1;

wire [5:0] add_6bits;
wire add_1bit;
wire [4:0]axis_to_address;
wire [3:0] assign_out;

reg [6:0] SRAM_loc_add_B_d1;

reg [5:0]  sour_X [0:14];
reg [5:0]  sour_Y [0:14];
reg [5:0]  sink_X [0:14];
reg [5:0]  sink_Y [0:14];

reg in_valid_d1;
reg wready_m_inf_d1;

reg [9:0] cnt1;
reg [9:0] temp_cnt;


reg [9:0] cnt3;
reg [6:0] cnt_dram;
reg [1:0] cnt_2; //cnt 0~3
wire [127:0] net_line;


reg [3:0] NETnum;
reg [1:0] Map [0:63][0:63];

reg [127:0] temp_reg;


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
parameter S7_WAVALID = 4'd7;
parameter S8_write_loc_D = 4'd8;
parameter S9_ = 4'd9;



reg [4:0] c_s, n_s, c_s_d;


//=====================================================================================================================================================================
reg  [6:0]  SRAM_loc_add_A, SRAM_loc_add_B;
reg  [127:0] SRAM_loc_in_A;
reg [127:0] SRAM_loc_out_A;
wire  [127:0] SRAM_loc_in_B;
reg [127:0] SRAM_loc_out_B;
reg SRAM_loc_WEAN; 
wire SRAM_loc_WEBN;		  
SJMA180_128X128X1BM1 SRAM_loc(.A0(SRAM_loc_add_A[0]),.A1(SRAM_loc_add_A[1]),.A2(SRAM_loc_add_A[2]),.A3(SRAM_loc_add_A[3]),.A4(SRAM_loc_add_A[4]),.A5(SRAM_loc_add_A[5]),.A6(SRAM_loc_add_A[6]),
                             .B0(SRAM_loc_add_B[0]),.B1(SRAM_loc_add_B[1]),.B2(SRAM_loc_add_B[2]),.B3(SRAM_loc_add_B[3]),.B4(SRAM_loc_add_B[4]),.B5(SRAM_loc_add_B[5]),.B6(SRAM_loc_add_B[6]),
                             .DOA0(SRAM_loc_out_A[0]),.DOA1(SRAM_loc_out_A[1]),.DOA2(SRAM_loc_out_A[2]),.DOA3(SRAM_loc_out_A[3]),.DOA4(SRAM_loc_out_A[4]),.DOA5(SRAM_loc_out_A[5]),.DOA6(SRAM_loc_out_A[6]),.DOA7(SRAM_loc_out_A[7]),
                             .DOA8(SRAM_loc_out_A[8]),.DOA9(SRAM_loc_out_A[9]),.DOA10(SRAM_loc_out_A[10]),.DOA11(SRAM_loc_out_A[11]),.DOA12(SRAM_loc_out_A[12]),.DOA13(SRAM_loc_out_A[13]),.DOA14(SRAM_loc_out_A[14]),.DOA15(SRAM_loc_out_A[15]),
                             .DOA16(SRAM_loc_out_A[16]),.DOA17(SRAM_loc_out_A[17]),.DOA18(SRAM_loc_out_A[18]),.DOA19(SRAM_loc_out_A[19]),.DOA20(SRAM_loc_out_A[20]),.DOA21(SRAM_loc_out_A[21]),.DOA22(SRAM_loc_out_A[22]),.DOA23(SRAM_loc_out_A[23]),
                             .DOA24(SRAM_loc_out_A[24]),.DOA25(SRAM_loc_out_A[25]),.DOA26(SRAM_loc_out_A[26]),.DOA27(SRAM_loc_out_A[27]),.DOA28(SRAM_loc_out_A[28]),.DOA29(SRAM_loc_out_A[29]),.DOA30(SRAM_loc_out_A[30]),.DOA31(SRAM_loc_out_A[31]),
                             .DOA32(SRAM_loc_out_A[32]),.DOA33(SRAM_loc_out_A[33]),.DOA34(SRAM_loc_out_A[34]),.DOA35(SRAM_loc_out_A[35]),.DOA36(SRAM_loc_out_A[36]),.DOA37(SRAM_loc_out_A[37]),.DOA38(SRAM_loc_out_A[38]),.DOA39(SRAM_loc_out_A[39]),
                             .DOA40(SRAM_loc_out_A[40]),.DOA41(SRAM_loc_out_A[41]),.DOA42(SRAM_loc_out_A[42]),.DOA43(SRAM_loc_out_A[43]),.DOA44(SRAM_loc_out_A[44]),.DOA45(SRAM_loc_out_A[45]),.DOA46(SRAM_loc_out_A[46]),.DOA47(SRAM_loc_out_A[47]),
                             .DOA48(SRAM_loc_out_A[48]),.DOA49(SRAM_loc_out_A[49]),.DOA50(SRAM_loc_out_A[50]),.DOA51(SRAM_loc_out_A[51]),.DOA52(SRAM_loc_out_A[52]),.DOA53(SRAM_loc_out_A[53]),.DOA54(SRAM_loc_out_A[54]),.DOA55(SRAM_loc_out_A[55]),
                             .DOA56(SRAM_loc_out_A[56]),.DOA57(SRAM_loc_out_A[57]),.DOA58(SRAM_loc_out_A[58]),.DOA59(SRAM_loc_out_A[59]),.DOA60(SRAM_loc_out_A[60]),.DOA61(SRAM_loc_out_A[61]),.DOA62(SRAM_loc_out_A[62]),.DOA63(SRAM_loc_out_A[63]),
                             .DOA64(SRAM_loc_out_A[64]),.DOA65(SRAM_loc_out_A[65]),.DOA66(SRAM_loc_out_A[66]),.DOA67(SRAM_loc_out_A[67]),.DOA68(SRAM_loc_out_A[68]),.DOA69(SRAM_loc_out_A[69]),.DOA70(SRAM_loc_out_A[70]),.DOA71(SRAM_loc_out_A[71]),
                             .DOA72(SRAM_loc_out_A[72]),.DOA73(SRAM_loc_out_A[73]),.DOA74(SRAM_loc_out_A[74]),.DOA75(SRAM_loc_out_A[75]),.DOA76(SRAM_loc_out_A[76]),.DOA77(SRAM_loc_out_A[77]),.DOA78(SRAM_loc_out_A[78]),.DOA79(SRAM_loc_out_A[79]),
                             .DOA80(SRAM_loc_out_A[80]),.DOA81(SRAM_loc_out_A[81]),.DOA82(SRAM_loc_out_A[82]),.DOA83(SRAM_loc_out_A[83]),.DOA84(SRAM_loc_out_A[84]),.DOA85(SRAM_loc_out_A[85]),.DOA86(SRAM_loc_out_A[86]),.DOA87(SRAM_loc_out_A[87]),
                             .DOA88(SRAM_loc_out_A[88]),.DOA89(SRAM_loc_out_A[89]),.DOA90(SRAM_loc_out_A[90]),.DOA91(SRAM_loc_out_A[91]),.DOA92(SRAM_loc_out_A[92]),.DOA93(SRAM_loc_out_A[93]),.DOA94(SRAM_loc_out_A[94]),.DOA95(SRAM_loc_out_A[95]),
                             .DOA96(SRAM_loc_out_A[96]),.DOA97(SRAM_loc_out_A[97]),.DOA98(SRAM_loc_out_A[98]),.DOA99(SRAM_loc_out_A[99]),.DOA100(SRAM_loc_out_A[100]),.DOA101(SRAM_loc_out_A[101]),.DOA102(SRAM_loc_out_A[102]),.DOA103(SRAM_loc_out_A[103]),
                             .DOA104(SRAM_loc_out_A[104]),.DOA105(SRAM_loc_out_A[105]),.DOA106(SRAM_loc_out_A[106]),.DOA107(SRAM_loc_out_A[107]),.DOA108(SRAM_loc_out_A[108]),.DOA109(SRAM_loc_out_A[109]),.DOA110(SRAM_loc_out_A[110]),.DOA111(SRAM_loc_out_A[111]),
                             .DOA112(SRAM_loc_out_A[112]),.DOA113(SRAM_loc_out_A[113]),.DOA114(SRAM_loc_out_A[114]),.DOA115(SRAM_loc_out_A[115]),.DOA116(SRAM_loc_out_A[116]),.DOA117(SRAM_loc_out_A[117]),.DOA118(SRAM_loc_out_A[118]),.DOA119(SRAM_loc_out_A[119]),
                             .DOA120(SRAM_loc_out_A[120]),.DOA121(SRAM_loc_out_A[121]),.DOA122(SRAM_loc_out_A[122]),.DOA123(SRAM_loc_out_A[123]),.DOA124(SRAM_loc_out_A[124]),.DOA125(SRAM_loc_out_A[125]),.DOA126(SRAM_loc_out_A[126]),.DOA127(SRAM_loc_out_A[127]),
                             .DOB0(SRAM_loc_out_B[0]),.DOB1(SRAM_loc_out_B[1]),.DOB2(SRAM_loc_out_B[2]),.DOB3(SRAM_loc_out_B[3]),.DOB4(SRAM_loc_out_B[4]),.DOB5(SRAM_loc_out_B[5]),.DOB6(SRAM_loc_out_B[6]),.DOB7(SRAM_loc_out_B[7]),
                             .DOB8(SRAM_loc_out_B[8]),.DOB9(SRAM_loc_out_B[9]),.DOB10(SRAM_loc_out_B[10]),.DOB11(SRAM_loc_out_B[11]),.DOB12(SRAM_loc_out_B[12]),.DOB13(SRAM_loc_out_B[13]),.DOB14(SRAM_loc_out_B[14]),.DOB15(SRAM_loc_out_B[15]),
                             .DOB16(SRAM_loc_out_B[16]),.DOB17(SRAM_loc_out_B[17]),.DOB18(SRAM_loc_out_B[18]),.DOB19(SRAM_loc_out_B[19]),.DOB20(SRAM_loc_out_B[20]),.DOB21(SRAM_loc_out_B[21]),.DOB22(SRAM_loc_out_B[22]),.DOB23(SRAM_loc_out_B[23]),
                             .DOB24(SRAM_loc_out_B[24]),.DOB25(SRAM_loc_out_B[25]),.DOB26(SRAM_loc_out_B[26]),.DOB27(SRAM_loc_out_B[27]),.DOB28(SRAM_loc_out_B[28]),.DOB29(SRAM_loc_out_B[29]),.DOB30(SRAM_loc_out_B[30]),.DOB31(SRAM_loc_out_B[31]),
                             .DOB32(SRAM_loc_out_B[32]),.DOB33(SRAM_loc_out_B[33]),.DOB34(SRAM_loc_out_B[34]),.DOB35(SRAM_loc_out_B[35]),.DOB36(SRAM_loc_out_B[36]),.DOB37(SRAM_loc_out_B[37]),.DOB38(SRAM_loc_out_B[38]),.DOB39(SRAM_loc_out_B[39]),
                             .DOB40(SRAM_loc_out_B[40]),.DOB41(SRAM_loc_out_B[41]),.DOB42(SRAM_loc_out_B[42]),.DOB43(SRAM_loc_out_B[43]),.DOB44(SRAM_loc_out_B[44]),.DOB45(SRAM_loc_out_B[45]),.DOB46(SRAM_loc_out_B[46]),.DOB47(SRAM_loc_out_B[47]),
                             .DOB48(SRAM_loc_out_B[48]),.DOB49(SRAM_loc_out_B[49]),.DOB50(SRAM_loc_out_B[50]),.DOB51(SRAM_loc_out_B[51]),.DOB52(SRAM_loc_out_B[52]),.DOB53(SRAM_loc_out_B[53]),.DOB54(SRAM_loc_out_B[54]),.DOB55(SRAM_loc_out_B[55]),
                             .DOB56(SRAM_loc_out_B[56]),.DOB57(SRAM_loc_out_B[57]),.DOB58(SRAM_loc_out_B[58]),.DOB59(SRAM_loc_out_B[59]),.DOB60(SRAM_loc_out_B[60]),.DOB61(SRAM_loc_out_B[61]),.DOB62(SRAM_loc_out_B[62]),.DOB63(SRAM_loc_out_B[63]),
                             .DOB64(SRAM_loc_out_B[64]),.DOB65(SRAM_loc_out_B[65]),.DOB66(SRAM_loc_out_B[66]),.DOB67(SRAM_loc_out_B[67]),.DOB68(SRAM_loc_out_B[68]),.DOB69(SRAM_loc_out_B[69]),.DOB70(SRAM_loc_out_B[70]),.DOB71(SRAM_loc_out_B[71]),
                             .DOB72(SRAM_loc_out_B[72]),.DOB73(SRAM_loc_out_B[73]),.DOB74(SRAM_loc_out_B[74]),.DOB75(SRAM_loc_out_B[75]),.DOB76(SRAM_loc_out_B[76]),.DOB77(SRAM_loc_out_B[77]),.DOB78(SRAM_loc_out_B[78]),.DOB79(SRAM_loc_out_B[79]),
                             .DOB80(SRAM_loc_out_B[80]),.DOB81(SRAM_loc_out_B[81]),.DOB82(SRAM_loc_out_B[82]),.DOB83(SRAM_loc_out_B[83]),.DOB84(SRAM_loc_out_B[84]),.DOB85(SRAM_loc_out_B[85]),.DOB86(SRAM_loc_out_B[86]),.DOB87(SRAM_loc_out_B[87]),
                             .DOB88(SRAM_loc_out_B[88]),.DOB89(SRAM_loc_out_B[89]),.DOB90(SRAM_loc_out_B[90]),.DOB91(SRAM_loc_out_B[91]),.DOB92(SRAM_loc_out_B[92]),.DOB93(SRAM_loc_out_B[93]),.DOB94(SRAM_loc_out_B[94]),.DOB95(SRAM_loc_out_B[95]),
                             .DOB96(SRAM_loc_out_B[96]),.DOB97(SRAM_loc_out_B[97]),.DOB98(SRAM_loc_out_B[98]),.DOB99(SRAM_loc_out_B[99]),.DOB100(SRAM_loc_out_B[100]),.DOB101(SRAM_loc_out_B[101]),.DOB102(SRAM_loc_out_B[102]),.DOB103(SRAM_loc_out_B[103]),
                             .DOB104(SRAM_loc_out_B[104]),.DOB105(SRAM_loc_out_B[105]),.DOB106(SRAM_loc_out_B[106]),.DOB107(SRAM_loc_out_B[107]),.DOB108(SRAM_loc_out_B[108]),.DOB109(SRAM_loc_out_B[109]),.DOB110(SRAM_loc_out_B[110]),.DOB111(SRAM_loc_out_B[111]),
                             .DOB112(SRAM_loc_out_B[112]),.DOB113(SRAM_loc_out_B[113]),.DOB114(SRAM_loc_out_B[114]),.DOB115(SRAM_loc_out_B[115]),.DOB116(SRAM_loc_out_B[116]),.DOB117(SRAM_loc_out_B[117]),.DOB118(SRAM_loc_out_B[118]),.DOB119(SRAM_loc_out_B[119]),
                             .DOB120(SRAM_loc_out_B[120]),.DOB121(SRAM_loc_out_B[121]),.DOB122(SRAM_loc_out_B[122]),.DOB123(SRAM_loc_out_B[123]),.DOB124(SRAM_loc_out_B[124]),.DOB125(SRAM_loc_out_B[125]),.DOB126(SRAM_loc_out_B[126]),.DOB127(SRAM_loc_out_B[127]),
                             .DIA0(SRAM_loc_in_A[0]),.DIA1(SRAM_loc_in_A[1]),.DIA2(SRAM_loc_in_A[2]),.DIA3(SRAM_loc_in_A[3]),.DIA4(SRAM_loc_in_A[4]),.DIA5(SRAM_loc_in_A[5]),.DIA6(SRAM_loc_in_A[6]),.DIA7(SRAM_loc_in_A[7]),
                             .DIA8(SRAM_loc_in_A[8]),.DIA9(SRAM_loc_in_A[9]),.DIA10(SRAM_loc_in_A[10]),.DIA11(SRAM_loc_in_A[11]),.DIA12(SRAM_loc_in_A[12]),.DIA13(SRAM_loc_in_A[13]),.DIA14(SRAM_loc_in_A[14]),.DIA15(SRAM_loc_in_A[15]),
                             .DIA16(SRAM_loc_in_A[16]),.DIA17(SRAM_loc_in_A[17]),.DIA18(SRAM_loc_in_A[18]),.DIA19(SRAM_loc_in_A[19]),.DIA20(SRAM_loc_in_A[20]),.DIA21(SRAM_loc_in_A[21]),.DIA22(SRAM_loc_in_A[22]),.DIA23(SRAM_loc_in_A[23]),
                             .DIA24(SRAM_loc_in_A[24]),.DIA25(SRAM_loc_in_A[25]),.DIA26(SRAM_loc_in_A[26]),.DIA27(SRAM_loc_in_A[27]),.DIA28(SRAM_loc_in_A[28]),.DIA29(SRAM_loc_in_A[29]),.DIA30(SRAM_loc_in_A[30]),.DIA31(SRAM_loc_in_A[31]),
                             .DIA32(SRAM_loc_in_A[32]),.DIA33(SRAM_loc_in_A[33]),.DIA34(SRAM_loc_in_A[34]),.DIA35(SRAM_loc_in_A[35]),.DIA36(SRAM_loc_in_A[36]),.DIA37(SRAM_loc_in_A[37]),.DIA38(SRAM_loc_in_A[38]),.DIA39(SRAM_loc_in_A[39]),
                             .DIA40(SRAM_loc_in_A[40]),.DIA41(SRAM_loc_in_A[41]),.DIA42(SRAM_loc_in_A[42]),.DIA43(SRAM_loc_in_A[43]),.DIA44(SRAM_loc_in_A[44]),.DIA45(SRAM_loc_in_A[45]),.DIA46(SRAM_loc_in_A[46]),.DIA47(SRAM_loc_in_A[47]),
                             .DIA48(SRAM_loc_in_A[48]),.DIA49(SRAM_loc_in_A[49]),.DIA50(SRAM_loc_in_A[50]),.DIA51(SRAM_loc_in_A[51]),.DIA52(SRAM_loc_in_A[52]),.DIA53(SRAM_loc_in_A[53]),.DIA54(SRAM_loc_in_A[54]),.DIA55(SRAM_loc_in_A[55]),
                             .DIA56(SRAM_loc_in_A[56]),.DIA57(SRAM_loc_in_A[57]),.DIA58(SRAM_loc_in_A[58]),.DIA59(SRAM_loc_in_A[59]),.DIA60(SRAM_loc_in_A[60]),.DIA61(SRAM_loc_in_A[61]),.DIA62(SRAM_loc_in_A[62]),.DIA63(SRAM_loc_in_A[63]),
                             .DIA64(SRAM_loc_in_A[64]),.DIA65(SRAM_loc_in_A[65]),.DIA66(SRAM_loc_in_A[66]),.DIA67(SRAM_loc_in_A[67]),.DIA68(SRAM_loc_in_A[68]),.DIA69(SRAM_loc_in_A[69]),.DIA70(SRAM_loc_in_A[70]),.DIA71(SRAM_loc_in_A[71]),
                             .DIA72(SRAM_loc_in_A[72]),.DIA73(SRAM_loc_in_A[73]),.DIA74(SRAM_loc_in_A[74]),.DIA75(SRAM_loc_in_A[75]),.DIA76(SRAM_loc_in_A[76]),.DIA77(SRAM_loc_in_A[77]),.DIA78(SRAM_loc_in_A[78]),.DIA79(SRAM_loc_in_A[79]),
                             .DIA80(SRAM_loc_in_A[80]),.DIA81(SRAM_loc_in_A[81]),.DIA82(SRAM_loc_in_A[82]),.DIA83(SRAM_loc_in_A[83]),.DIA84(SRAM_loc_in_A[84]),.DIA85(SRAM_loc_in_A[85]),.DIA86(SRAM_loc_in_A[86]),.DIA87(SRAM_loc_in_A[87]),
                             .DIA88(SRAM_loc_in_A[88]),.DIA89(SRAM_loc_in_A[89]),.DIA90(SRAM_loc_in_A[90]),.DIA91(SRAM_loc_in_A[91]),.DIA92(SRAM_loc_in_A[92]),.DIA93(SRAM_loc_in_A[93]),.DIA94(SRAM_loc_in_A[94]),.DIA95(SRAM_loc_in_A[95]),
                             .DIA96(SRAM_loc_in_A[96]),.DIA97(SRAM_loc_in_A[97]),.DIA98(SRAM_loc_in_A[98]),.DIA99(SRAM_loc_in_A[99]),.DIA100(SRAM_loc_in_A[100]),.DIA101(SRAM_loc_in_A[101]),.DIA102(SRAM_loc_in_A[102]),.DIA103(SRAM_loc_in_A[103]),
                             .DIA104(SRAM_loc_in_A[104]),.DIA105(SRAM_loc_in_A[105]),.DIA106(SRAM_loc_in_A[106]),.DIA107(SRAM_loc_in_A[107]),.DIA108(SRAM_loc_in_A[108]),.DIA109(SRAM_loc_in_A[109]),.DIA110(SRAM_loc_in_A[110]),.DIA111(SRAM_loc_in_A[111]),
                             .DIA112(SRAM_loc_in_A[112]),.DIA113(SRAM_loc_in_A[113]),.DIA114(SRAM_loc_in_A[114]),.DIA115(SRAM_loc_in_A[115]),.DIA116(SRAM_loc_in_A[116]),.DIA117(SRAM_loc_in_A[117]),.DIA118(SRAM_loc_in_A[118]),.DIA119(SRAM_loc_in_A[119]),
                             .DIA120(SRAM_loc_in_A[120]),.DIA121(SRAM_loc_in_A[121]),.DIA122(SRAM_loc_in_A[122]),.DIA123(SRAM_loc_in_A[123]),.DIA124(SRAM_loc_in_A[124]),.DIA125(SRAM_loc_in_A[125]),.DIA126(SRAM_loc_in_A[126]),.DIA127(SRAM_loc_in_A[127]),
                             .DIB0(SRAM_loc_in_B[0]),.DIB1(SRAM_loc_in_B[1]),.DIB2(SRAM_loc_in_B[2]),.DIB3(SRAM_loc_in_B[3]),.DIB4(SRAM_loc_in_B[4]),.DIB5(SRAM_loc_in_B[5]),.DIB6(SRAM_loc_in_B[6]),.DIB7(SRAM_loc_in_B[7]),
                             .DIB8(SRAM_loc_in_B[8]),.DIB9(SRAM_loc_in_B[9]),.DIB10(SRAM_loc_in_B[10]),.DIB11(SRAM_loc_in_B[11]),.DIB12(SRAM_loc_in_B[12]),.DIB13(SRAM_loc_in_B[13]),.DIB14(SRAM_loc_in_B[14]),.DIB15(SRAM_loc_in_B[15]),
                             .DIB16(SRAM_loc_in_B[16]),.DIB17(SRAM_loc_in_B[17]),.DIB18(SRAM_loc_in_B[18]),.DIB19(SRAM_loc_in_B[19]),.DIB20(SRAM_loc_in_B[20]),.DIB21(SRAM_loc_in_B[21]),.DIB22(SRAM_loc_in_B[22]),.DIB23(SRAM_loc_in_B[23]),
                             .DIB24(SRAM_loc_in_B[24]),.DIB25(SRAM_loc_in_B[25]),.DIB26(SRAM_loc_in_B[26]),.DIB27(SRAM_loc_in_B[27]),.DIB28(SRAM_loc_in_B[28]),.DIB29(SRAM_loc_in_B[29]),.DIB30(SRAM_loc_in_B[30]),.DIB31(SRAM_loc_in_B[31]),
                             .DIB32(SRAM_loc_in_B[32]),.DIB33(SRAM_loc_in_B[33]),.DIB34(SRAM_loc_in_B[34]),.DIB35(SRAM_loc_in_B[35]),.DIB36(SRAM_loc_in_B[36]),.DIB37(SRAM_loc_in_B[37]),.DIB38(SRAM_loc_in_B[38]),.DIB39(SRAM_loc_in_B[39]),
                             .DIB40(SRAM_loc_in_B[40]),.DIB41(SRAM_loc_in_B[41]),.DIB42(SRAM_loc_in_B[42]),.DIB43(SRAM_loc_in_B[43]),.DIB44(SRAM_loc_in_B[44]),.DIB45(SRAM_loc_in_B[45]),.DIB46(SRAM_loc_in_B[46]),.DIB47(SRAM_loc_in_B[47]),
                             .DIB48(SRAM_loc_in_B[48]),.DIB49(SRAM_loc_in_B[49]),.DIB50(SRAM_loc_in_B[50]),.DIB51(SRAM_loc_in_B[51]),.DIB52(SRAM_loc_in_B[52]),.DIB53(SRAM_loc_in_B[53]),.DIB54(SRAM_loc_in_B[54]),.DIB55(SRAM_loc_in_B[55]),
                             .DIB56(SRAM_loc_in_B[56]),.DIB57(SRAM_loc_in_B[57]),.DIB58(SRAM_loc_in_B[58]),.DIB59(SRAM_loc_in_B[59]),.DIB60(SRAM_loc_in_B[60]),.DIB61(SRAM_loc_in_B[61]),.DIB62(SRAM_loc_in_B[62]),.DIB63(SRAM_loc_in_B[63]),
                             .DIB64(SRAM_loc_in_B[64]),.DIB65(SRAM_loc_in_B[65]),.DIB66(SRAM_loc_in_B[66]),.DIB67(SRAM_loc_in_B[67]),.DIB68(SRAM_loc_in_B[68]),.DIB69(SRAM_loc_in_B[69]),.DIB70(SRAM_loc_in_B[70]),.DIB71(SRAM_loc_in_B[71]),
                             .DIB72(SRAM_loc_in_B[72]),.DIB73(SRAM_loc_in_B[73]),.DIB74(SRAM_loc_in_B[74]),.DIB75(SRAM_loc_in_B[75]),.DIB76(SRAM_loc_in_B[76]),.DIB77(SRAM_loc_in_B[77]),.DIB78(SRAM_loc_in_B[78]),.DIB79(SRAM_loc_in_B[79]),
                             .DIB80(SRAM_loc_in_B[80]),.DIB81(SRAM_loc_in_B[81]),.DIB82(SRAM_loc_in_B[82]),.DIB83(SRAM_loc_in_B[83]),.DIB84(SRAM_loc_in_B[84]),.DIB85(SRAM_loc_in_B[85]),.DIB86(SRAM_loc_in_B[86]),.DIB87(SRAM_loc_in_B[87]),
                             .DIB88(SRAM_loc_in_B[88]),.DIB89(SRAM_loc_in_B[89]),.DIB90(SRAM_loc_in_B[90]),.DIB91(SRAM_loc_in_B[91]),.DIB92(SRAM_loc_in_B[92]),.DIB93(SRAM_loc_in_B[93]),.DIB94(SRAM_loc_in_B[94]),.DIB95(SRAM_loc_in_B[95]),
                             .DIB96(SRAM_loc_in_B[96]),.DIB97(SRAM_loc_in_B[97]),.DIB98(SRAM_loc_in_B[98]),.DIB99(SRAM_loc_in_B[99]),.DIB100(SRAM_loc_in_B[100]),.DIB101(SRAM_loc_in_B[101]),.DIB102(SRAM_loc_in_B[102]),.DIB103(SRAM_loc_in_B[103]),
                             .DIB104(SRAM_loc_in_B[104]),.DIB105(SRAM_loc_in_B[105]),.DIB106(SRAM_loc_in_B[106]),.DIB107(SRAM_loc_in_B[107]),.DIB108(SRAM_loc_in_B[108]),.DIB109(SRAM_loc_in_B[109]),.DIB110(SRAM_loc_in_B[110]),.DIB111(SRAM_loc_in_B[111]),
                             .DIB112(SRAM_loc_in_B[112]),.DIB113(SRAM_loc_in_B[113]),.DIB114(SRAM_loc_in_B[114]),.DIB115(SRAM_loc_in_B[115]),.DIB116(SRAM_loc_in_B[116]),.DIB117(SRAM_loc_in_B[117]),.DIB118(SRAM_loc_in_B[118]),.DIB119(SRAM_loc_in_B[119]),
                             .DIB120(SRAM_loc_in_B[120]),.DIB121(SRAM_loc_in_B[121]),.DIB122(SRAM_loc_in_B[122]),.DIB123(SRAM_loc_in_B[123]),.DIB124(SRAM_loc_in_B[124]),.DIB125(SRAM_loc_in_B[125]),.DIB126(SRAM_loc_in_B[126]),.DIB127(SRAM_loc_in_B[127]),
                             .WEAN(SRAM_loc_WEAN),.WEBN(SRAM_loc_WEBN),.CKA(clk),.CKB(clk),.CSA(1'b1),.CSB(1'b1),.OEA(1'b1),.OEB(1'b1));

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
        // if(exe_net_ID == NETnum)    n_s = S8_write_loc_D ; 
        // else                          n_s = S5_fillMAP;     	
                         n_s = S5_fillMAP;     	
		  S5_fillMAP:
        if (flag_1)
            n_s = S6_RETRACE;
        else
            n_s = S5_fillMAP ;	
		  S6_RETRACE:
        if (cur_X_d1==sour_X[exe_net] && cur_Y_d1==sour_Y[exe_net])
            n_s = S7_WAVALID;
        else
            n_s = S6_RETRACE ;	
		  // S7_write_loc_S:
      //   if (arready_m_inf)
      //       n_s = S4_setGoal;
      //   else
      //       n_s = S7_write_loc_S ;
      S7_WAVALID:
            if (awready_m_inf)
              n_s = S8_write_loc_D;
            else
              n_s = S7_WAVALID;											
		  S8_write_loc_D :
          if (bvalid_m_inf) n_s = S0_IDLE;
          else              n_s = S8_write_loc_D ;
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
  if (!rst_n) cnt1 <= 0;
  else if(in_valid && flag_3) begin
    cnt1 <= cnt1 + 1;
  end
  else if(c_s==S0_IDLE) cnt1 <= 0;
end

// ===============================================================
//  				      cnt_2
// ===============================================================
  always @ (posedge clk or negedge rst_n) begin 
  	if (!rst_n) cnt_2 <= 0 ;
  	else begin
  		if (c_s == S0_IDLE)                                                    cnt_2 <= 0 ;
  		else if(c_s == S4_setGoal || (c_s == S5_fillMAP && n_s == S5_fillMAP)) cnt_2 <= cnt_2 + 1 ;
  		else if(c_s == S5_fillMAP && n_s == S6_RETRACE)                        cnt_2 <= cnt_2 - 2 ;
  		else if(c_s == S6_RETRACE)                                             cnt_2 <= cnt_2 - 1 ;
  		// else if(c_s == S7_WAVALID && n_s == S4_setGoal)                    cnt_2 <= 0 ;      
  		else cnt_2 <= cnt_2 ;
  	end
  end
  always @ (posedge clk or negedge rst_n) begin 
  	if (!rst_n) cnt3 <= 0 ;  
    else if(in_valid) cnt3 <= cnt3 + 1;
    else if(c_s==S0_IDLE) cnt3 <= 0;
    else cnt3 <= cnt3;
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
      if(c_s==S8_write_loc_D) temp_cnt <= temp_cnt + 1;
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
  else if(c_s==S5_fillMAP)begin
    if(Map[sink_Y[exe_net]+1][sink_X[exe_net]]!=0 && Map[sink_Y[exe_net]-1][sink_X[exe_net]]!=0 && Map[sink_Y[exe_net]][sink_X[exe_net]+1]!=0 && Map[sink_Y[exe_net]][sink_X[exe_net]-1]!=0) flag_1 <= 1;
    else flag_1 <= 0;  
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
    else if (in_valid && cnt1[0]==1) NETnum <= NETnum+1;
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
      if(in_valid) net_ID[cnt3] <= net_id;
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
        else if(in_valid && !flag_3) sour_X[cnt1] <= loc_x;
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
        else if(in_valid && !flag_3) sour_Y[cnt1] <= loc_y;
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
        else if(in_valid && flag_3) sink_X[cnt1] <= loc_x;
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
        else if(in_valid && flag_3) sink_Y[cnt1] <= loc_y;
        else begin
          for (i=0; i<15; i=i+1) begin
            sink_Y[i] <= sink_Y[i];  
          end
        end 
      end
  
// ===============================================================
//      Calculate
// ===============================================================

  always@(posedge clk or negedge rst_n)
    begin
      if(!rst_n) exe_net <= 0;
      // else if(SRAM) exe_net <= exe_net + 1;
      else exe_net <= exe_net;
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
          Map[sour_Y[exe_net]][sour_X[exe_net]] <= 2; // propagate from source
          Map[sink_Y[exe_net]][sink_X[exe_net]] <= 0; // sink is the goal of propagation
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
      else if(c_s==S6_RETRACE)
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
        cur_X <= sink_X[exe_net];
        cur_Y <= sink_Y[exe_net];
      end
      else if(c_s == S6_RETRACE) begin
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
  always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
      cur_X_d2 <= 0;
      cur_Y_d2 <= 0;
    end
    else begin
      cur_X_d2 <= cur_X_d1;
      cur_Y_d2 <= cur_Y_d1;
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
    else if(c_s==S7_WAVALID) awvalid_m_inf <= 1;
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
    else if(wready_m_inf) r_wdata <= SRAM_loc_out_B;
    else r_wdata <= SRAM_loc_in_A;
  end
always@(posedge clk or negedge rst_n)
  begin
    if(!rst_n) temp_reg <= 0;
    else if(c_s==S8_write_loc_D && SRAM_loc_add_B) temp_reg <= SRAM_loc_out_B;
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
  always@(posedge clk or negedge rst_n) //SRAM_loc_WEAN
    begin
      if(!rst_n) SRAM_loc_add_B_d1 <= 0;
      else SRAM_loc_add_B_d1 <= SRAM_loc_add_B;
    end
 // ---------- A --------------------------
  always@(posedge clk or negedge rst_n) //SRAM_loc_WEAN
    begin
      if(!rst_n) SRAM_loc_WEAN <= 0;
      else if(c_s==S4_setGoal) SRAM_loc_WEAN <= 1;
      else if(rvalid_m_inf_d1) begin
        if(c_s==S3_read_weiMAP_D)  SRAM_loc_WEAN <= 1;
        else                    SRAM_loc_WEAN <= 0;
      end
      else if((c_s==S6_RETRACE || c_s==S7_WAVALID) && flag_2)begin
        if(cur_X==cur_X_d2 && cur_Y==cur_Y_d2) SRAM_loc_WEAN <= 1;
        else  SRAM_loc_WEAN <= 0;
      end
      else SRAM_loc_WEAN <= 1;
    end
  always@(posedge clk or negedge rst_n) //SRAM128_add
    begin // SRAM_loc_add_A [6:0]
      if(!rst_n)  SRAM_loc_add_A <= 0;
      else if(c_s==S5_fillMAP) SRAM_loc_add_A <= {add_6bits,add_1bit};
      else if(c_s==S6_RETRACE) SRAM_loc_add_A <= SRAM_loc_add_B_d1;
      else if(!SRAM_loc_WEAN)  SRAM_loc_add_A <= SRAM_loc_add_A+1;
      else SRAM_loc_add_A <= 0;
    end


  always@(posedge clk or negedge rst_n) //SRAM128_in
    begin
      if(!rst_n)  SRAM_loc_in_A <= 0;
      else if(rvalid_m_inf_d1) SRAM_loc_in_A <= r_rdata;
      else if(c_s==S6_RETRACE)begin
        if(SRAM_loc_add_A==SRAM_loc_add_B_d1) SRAM_loc_in_A <= SRAM_loc_in_A + net_line;
        else                          SRAM_loc_in_A <= SRAM_loc_out_B + net_line;                 
      end
      else if(c_s==S7_WAVALID)  SRAM_loc_in_A <= SRAM_loc_out_B;
      else SRAM_loc_in_A <= SRAM_loc_in_A;
    end
 // ---------- B --------------------------
 // -------- only read in B channel -------
  assign SRAM_loc_WEBN = 1'b1;
  always@(posedge clk or negedge rst_n)
    begin
      if(!rst_n)  SRAM_loc_add_B <= 0;
      else if(c_s==S3_read_weiMAP_D)begin
        if(SRAM_loc_add_B==127) SRAM_loc_add_B <= SRAM_loc_add_B;  
        else                    SRAM_loc_add_B <= SRAM_loc_add_B+1;
      end
      else if(c_s==S5_fillMAP||c_s==S6_RETRACE) SRAM_loc_add_B <= {add_6bits,add_1bit};
      else if(wready_m_inf) SRAM_loc_add_B <= SRAM_loc_add_B+1;
      else if(c_s==S8_write_loc_D) SRAM_loc_add_B <= 2;
      else if(n_s==S8_write_loc_D) SRAM_loc_add_B <= 1;
      else if(c_s==S7_WAVALID) SRAM_loc_add_B <= 0;
      
      else SRAM_loc_add_B <= SRAM_loc_add_B;
    end
  

// ======== SRAM_wei ===================================
  assign add_1bit  = (cur_X>31)? 1 : 0;
  assign add_6bits = cur_Y;
 // ---------- A --------------------------
  always@(posedge clk or negedge rst_n) //SRAM_wei_WEB
    begin
      if(!rst_n) SRAM_wei_WEB <= 0;
      else if(rvalid_m_inf_d1) begin
        if(c_s==S3_read_weiMAP_D) SRAM_wei_WEB <= 0;
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
assign axis_to_address = (cur_X_d2>31)? cur_X_d2-'d32 : cur_X_d2;
assign assign_out = SRAM_wei_out >> (axis_to_address*4);
assign net_line = net_ID[exe_net] << (axis_to_address*4); 
  

always@(posedge clk or negedge rst_n)
begin
  if(!rst_n)  flag_2 <= 0;
  else if(c_s==S6_RETRACE && (cur_X_d1!=sink_X[exe_net] || cur_Y_d1!=sink_Y[exe_net])) flag_2 <= 1;
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
    else if(c_s==S6_RETRACE) begin
      // if(cur_X==sour_X && cur_Y==sour_Y) cost <= cost;
      // if(cur_X_d1!=sink_X[exe_net] || cur_Y_d1!=sink_Y[exe_net]) cost <= cost + SRAM_wei_out[axis_to_address*4 +: 4];
      if(flag_2) cost <= cost + assign_out;
    end
    else cost <= cost;
  end


endmodule
