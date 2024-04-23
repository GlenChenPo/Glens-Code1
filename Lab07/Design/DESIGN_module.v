module CLK_1_MODULE (
    clk,
    rst_n,
    in_valid,
	in_matrix_A,
    in_matrix_B,
    out_idle,
    handshake_sready,
    handshake_din,

    flag_handshake_to_clk1,
    flag_clk1_to_handshake,

	fifo_empty,
    fifo_rdata,
    fifo_rinc,
    out_valid,
    out_matrix,

    flag_clk1_to_fifo,
    flag_fifo_to_clk1
);
input clk;
input rst_n;
input in_valid;
input [3:0] in_matrix_A;
input [3:0] in_matrix_B;
input out_idle;
output reg handshake_sready;
output reg [7:0] handshake_din;
// You can use the the custom flag ports for your design
input  flag_handshake_to_clk1;
output flag_clk1_to_handshake;

input fifo_empty;
input [7:0] fifo_rdata;
output fifo_rinc;
output reg out_valid;
output reg [7:0] out_matrix;
// You can use the the custom flag ports for your design
output flag_clk1_to_fifo;
input flag_fifo_to_clk1;

//===============================================================
//         Reg & Wire Declaration
//===============================================================
reg [9:0] cnt;
reg [9:0] cnt_matrix;

reg [3:0] matrix_32 [0:31];
reg fifo_rinc;

reg [7:0] matrix_mult [0:255];

// delay
reg flag_handshake_to_clk1_d;
reg in_valid_d;
reg fifo_empty_d;
reg flag_fifo_to_clk1_d;

integer i,j,k;
//======================================================
//               STATE
//======================================================
parameter S_IDLE      = 3'd0;
parameter S_INPUT     = 3'd1; 
parameter S_GIVE_MAT  = 3'd2; 
parameter S_GET_MAT   = 3'd3; 
parameter S_OUTPUT    = 3'd4; 

reg [2:0] c_s,n_s;

//===============================================================
//                current state
//===============================================================
always @(posedge clk or negedge rst_n)
  begin
      if (!rst_n)
          c_s <= S_IDLE;
      else
          c_s <= n_s;
  end
//===============================================================
//                   next state
//===============================================================
always @(*)
  begin
    case (c_s)
        S_IDLE:
          begin
            if (in_valid)
                n_s = S_INPUT;
            else
                n_s = S_IDLE;
          end
        S_INPUT:
          begin
            if (in_valid==0)
                n_s = S_GIVE_MAT;
            else
                n_s = S_INPUT;
          end   
        S_GIVE_MAT:
          begin
            if (cnt_matrix==32)
                n_s = S_GET_MAT;
            else
                n_s = S_GIVE_MAT;
          end        
        S_GET_MAT:
          begin
            if (cnt_matrix==256)
                n_s = S_OUTPUT;
            else
                n_s = S_GET_MAT;
          end
        S_OUTPUT:
          begin
            if (cnt_matrix==256)
                n_s = S_IDLE;
            else
                n_s = S_OUTPUT;            
          end                           
        default:
            n_s = S_IDLE;
    endcase
  end
// ===============================================================
//       Counter
// ===============================================================
always @(posedge clk or negedge rst_n) begin // cnt
  if(!rst_n) cnt <= 0;
  else cnt <= cnt_matrix;
end
always @(posedge clk or negedge rst_n) begin // cnt
  if (!rst_n)          cnt_matrix <= 0;
  else if (c_s==S_GIVE_MAT) begin
    if (cnt_matrix==32) cnt_matrix <= 0;
    else if(flag_handshake_to_clk1 && !flag_handshake_to_clk1_d) cnt_matrix <= cnt_matrix + 1;
    else                       cnt_matrix <= cnt_matrix;
  end  
  else if(c_s==S_GET_MAT)begin
    if (cnt_matrix==256) cnt_matrix <= 0;
    else if (flag_fifo_to_clk1!= flag_fifo_to_clk1_d) cnt_matrix <= cnt_matrix+1;
    else           cnt_matrix <= cnt_matrix;
  end
  else if(c_s==S_OUTPUT)begin
    cnt_matrix <= cnt_matrix+1;
  end
  else  cnt_matrix <= 0;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) flag_fifo_to_clk1_d <= 0;
  else flag_fifo_to_clk1_d <= flag_fifo_to_clk1;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) flag_handshake_to_clk1_d <= 0;
  else flag_handshake_to_clk1_d <= flag_handshake_to_clk1;
end
//---------- DELAY ------------------------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) fifo_empty_d<=0;
  else fifo_empty_d <= fifo_empty;
end
// ===============================================================
// Design
// ===============================================================
//------ PATTERN and ModuleA -------------------------
//-----INPUT-----------------------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    for(i=0;i<32;i=i+1) begin
      matrix_32[i] <= 0;
    end
  end
  else if (in_valid) begin
    matrix_32[31]<=in_matrix_B;
    matrix_32[15]<=in_matrix_A;
    for(i=16;i<31;i=i+1) begin
      matrix_32[i] <= matrix_32[i+1];
    end
    for(j=0;j<15;j=j+1) begin
      matrix_32[j] <= matrix_32[j+1];
    end
  end
  else begin
    for(i=0;i<32;i=i+1) begin
      matrix_32[i] <= matrix_32[i];
    end  
  end
end
//-----OUTPUT----------------------------------------
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) out_matrix <= 0;
  else begin
    if (cnt_matrix==256)        out_matrix <= 0; 
    else if (c_s==S_OUTPUT)     out_matrix <= matrix_mult[cnt_matrix];
    else                  out_matrix <= out_matrix;
  end
end
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) out_valid <= 0;
  else begin
    if (cnt_matrix==256)        out_valid <= 0; 
    else if (c_s==S_OUTPUT)     out_valid <= 1;
    else                  out_valid <= out_valid;
  end
end

//------ ModuleA and ModulesB-------------------------
always @(posedge clk or negedge rst_n) begin
  if(!rst_n)  handshake_din <= 0;
  else if(c_s==S_GIVE_MAT) handshake_din <= matrix_32[cnt_matrix];
  else                     handshake_din <= 0;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n)               handshake_sready <= 0;
  else if(c_s==S_GIVE_MAT) handshake_sready <= 1;
  else                     handshake_sready <= 0;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n)              fifo_rinc <= 0;
  else if(c_s==S_GET_MAT) fifo_rinc <= 1;
  else if(c_s==S_OUTPUT)  fifo_rinc <= 0;
  else                    fifo_rinc <= fifo_rinc;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    for(i=0 ; i<256 ; i=i+1)begin
      matrix_mult[i] <= 0;
    end
  end
  else if(c_s==S_GET_MAT) begin
     matrix_mult[cnt] <= fifo_rdata;
  end
  else if(c_s==S_IDLE) begin
    for(i=0 ; i<256 ; i=i+1)begin
      matrix_mult[i] <= 0;
    end  
  end
  else begin
    for(i=0 ; i<256 ; i=i+1)begin
      matrix_mult[i] <= matrix_mult[i];
    end  
  end  
end

endmodule
//=======================================================================
//=======================================================================
module CLK_2_MODULE (
    clk,
    rst_n,
    in_valid,
    fifo_full,
    in_matrix,
    out_valid,
    out_matrix,
    busy,

    flag_handshake_to_clk2,
    flag_clk2_to_handshake,

    flag_fifo_to_clk2,
    flag_clk2_to_fifo
);

input clk;
input rst_n;
input in_valid;
input fifo_full;
input [7:0] in_matrix;
output reg out_valid;
output reg [7:0] out_matrix;
output reg busy;

// You can use the the custom flag ports for your design
input  flag_handshake_to_clk2;
output flag_clk2_to_handshake;

input  flag_fifo_to_clk2;
output flag_clk2_to_fifo;
//===============================================================
//         Reg & Wire Declaration
//===============================================================
reg [9:0] cnt;
reg [9:0] cnt_input;
reg in_valid_d;
reg [7:0] matrix_A [0:15];
reg [7:0] matrix_B [0:15];
reg [7:0] matrix_mult [0:255];

integer i,j,k;
//======================================================
//               STATE
//======================================================
parameter S_IDLE     = 3'd0;
parameter S_GET_MAT  = 3'd1; 
parameter S_CAL      = 3'd2; 
parameter S_GIVE_MAT = 3'd3; 
reg [2:0] c_s,n_s;

//===============================================================
//                current state
//===============================================================
always @(posedge clk or negedge rst_n)
  begin
      if (!rst_n)
          c_s <= S_IDLE;
      else
          c_s <= n_s;
  end
//===============================================================
//                   next state
//===============================================================
always @(*)
  begin
      case (c_s)
          S_IDLE:
            begin
              if (in_valid)
                  n_s = S_GET_MAT;
              else
                  n_s = S_IDLE;
            end
          S_GET_MAT:
            begin
              if (cnt_input==32)
                  n_s = S_CAL;
              else
                  n_s = S_GET_MAT;
            end  
          S_CAL:
            begin
              n_s = S_GIVE_MAT;
            end           
          S_GIVE_MAT:
            begin
              if (cnt_input==256 && !fifo_full)
                  n_s = S_IDLE;
              else
                  n_s = S_GIVE_MAT;
            end  
          default:
              n_s = S_IDLE;
      endcase
  end
// ===============================================================
//       Counter
// ===============================================================
always @(posedge clk or negedge rst_n) begin // cnt
  if (!rst_n)          cnt <= 0;
  else if (in_valid)   cnt <= cnt + 1;
  else if (c_s==S_CAL & cnt==10)  cnt <= 0;
  else                 cnt <= 0;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) cnt_input <= 0;
  else if(c_s==S_IDLE)             cnt_input <= 0;
  else if(!in_valid && in_valid_d) cnt_input <= cnt_input+1;
  else if(c_s == S_CAL)  cnt_input <= 0;
  else if(c_s==S_GIVE_MAT) begin
    if(fifo_full)  cnt_input <= cnt_input;
    else           cnt_input <= cnt_input + 1;
  end
  else                             cnt_input <= cnt_input;
end
always @(posedge clk or negedge rst_n) begin
  if(!rst_n) in_valid_d <= 0;
  else in_valid_d <= in_valid;
end
// ===============================================================
// Design
// ===============================================================
//------- INPUT ----------------------------------
  always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    for (i=0 ; i<16 ; i=i+1) begin
      matrix_A [i] <= 0;
    end
  end
  else if(cnt_input<16) begin
    if(in_valid && !in_valid_d)begin
      matrix_A[15] <= in_matrix;
      for(i=0 ; i<15 ; i=i+1)begin
        matrix_A [i] <= matrix_A [i+1];
      end      
    end  
  end
  else begin
    for (i=0 ; i<16 ; i=i+1) begin
      matrix_A [i] <= matrix_A [i];
    end
  end
  end
  always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
    for (i=0 ; i<16 ; i=i+1) begin
      matrix_B [i] <= 0;
    end
  end
  else if (c_s==S_GET_MAT)begin
    if(cnt_input<32) begin
      if(in_valid && !in_valid_d)begin        
        matrix_B[15] <= in_matrix;
        for(i=0 ; i<15 ; i=i+1)begin
          matrix_B [i] <= matrix_B [i+1];
        end
      end
    end
  end
  else begin
    for (i=0 ; i<16 ; i=i+1) begin
      matrix_B [i] <= matrix_B [i];
    end
  end
  end
//------- Calculation ----------------------------
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      for (j=0 ; j<256 ; j=j+1) begin
          matrix_mult[j] <= 0;
      end
    end
    else if (c_s==S_CAL)begin
      for (j=0 ; j<16 ; j=j+1) begin
        for (i=0 ; i<16 ; i=i+1) begin
          matrix_mult[16*j+i] <= matrix_A[j]*matrix_B[i];    
        end
      end
    end
    else begin
      for (j=0 ; j<256 ; j=j+1) begin
          matrix_mult[j] <= matrix_mult[j];
      end  
    end
  end
//------- busy -----------------------------------
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) busy <= 0;
    else if(c_s==S_GET_MAT) busy <= 0;
    else if(c_s==S_CAL)     busy <= 1;
    else                    busy <= 0;
  end
//------- OUTPUT ---------------------------------
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) out_valid <= 0;
    else if(cnt_input==256 && !fifo_full)  out_valid <= 0;
    else if(c_s==S_GIVE_MAT) out_valid <= 1;
    else                     out_valid <= out_valid;
  end
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) out_matrix <= 0;
    else if(cnt_input==256 && !fifo_full)  out_matrix <= 0;
    else if(c_s==S_GIVE_MAT) begin
      if(fifo_full)  out_matrix <= out_matrix;
      else           out_matrix <= matrix_mult[cnt_input];
        
    end
    else  out_matrix <= out_matrix;
  end  
endmodule