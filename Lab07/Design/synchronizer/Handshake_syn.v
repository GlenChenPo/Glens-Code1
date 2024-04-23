module Handshake_syn #(parameter WIDTH=8) (
    sclk,
    dclk,
    rst_n,
    sready,
    din,
    dbusy,
    sidle,
    dvalid,
    dout,

    flag_handshake_to_clk1,
    flag_clk1_to_handshake,

    flag_handshake_to_clk2,
    flag_clk2_to_handshake
);

input sclk, dclk;
input rst_n;
input sready;
input [WIDTH-1:0] din;
input dbusy;
output sidle;
output reg dvalid;
output reg [WIDTH-1:0] dout;

// You can change the input / output of the custom flag ports
output reg flag_handshake_to_clk1;
input flag_clk1_to_handshake;

output flag_handshake_to_clk2;
input flag_clk2_to_handshake;

// Remember:
//   Don't modify the signal name
reg sreq;
wire dreq;
reg dack;
wire sack;
// -------------------------------------

reg [WIDTH-1:0] s_data;


// two NDFF_syn sclk to dclk & dclk to sclk
NDFF_syn ndff0(.D(sreq), .Q(dreq), .clk(dclk), .rst_n(rst_n));
NDFF_syn ndff1(.D(dack), .Q(sack), .clk(sclk), .rst_n(rst_n));


//------------ DATA ----------------------
always @(posedge sclk or negedge rst_n) 
begin
  if (!rst_n)      s_data <= 0;
  else if (sready) s_data <= din;
  else             s_data <= s_data;
end 
always @(posedge dclk or negedge rst_n) 
begin
  if (!rst_n)              dout <= 0;
  else if (!dbusy && dreq) dout <= s_data;
  else                     dout <= dout;
end
always @(posedge dclk or negedge rst_n) 
begin
  if (!rst_n)              dvalid <= 0;
  else if (!dbusy && dreq) dvalid <= 1;
  else                     dvalid <= 0;
end
//-------Src and Dest control-------------
always @(posedge sclk or negedge rst_n) 
begin
  if(!rst_n)       sreq <= 0; 
  else if (sack)   sreq <= 0;
  else if (sready) sreq <= 1;
  else             sreq <= sreq;
end
always @(posedge dclk or negedge rst_n) 
begin
  if (!rst_n)      dack <= 0;
  else if (dreq)   dack <= 1;
  else             dack <= 0;
end
//------- SIDLE -------------
assign sidle = (sreq || sack)? 0:1 ;

//------- FLAG --------------
assign flag_handshake_to_clk1 = sack;
endmodule