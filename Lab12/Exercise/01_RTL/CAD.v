module CAD(
    //Input Port
    clk,
    rst_n,
    in_valid,
    in_valid2,
    mode,
    matrix_size,
	matrix,
    matrix_idx,

    //Output Port
    out_valid,
    out_value
    );
//---------------------------------------------------------------------
//   IN & OUT
//---------------------------------------------------------------------
input clk, rst_n, in_valid, in_valid2, mode;
input [1:0] matrix_size;
input [7:0] matrix;
input [3:0] matrix_idx;

output reg out_valid;
output reg out_value;

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------
parameter	MP_IDLE     = 'd0,
            MP_UP 	    = 'd1,
            MP_WAIT     = 'd2,
		    MP_DOWN     = 'd3;

parameter	ST_IDLE 	= 'd0,
		    ST_STORAGE  = 'd1,
		    ST_KERNEL   = 'd2,
            ST_IDX      = 'd3,
			ST_CONV 	= 'd4,
			ST_DECONV	= 'd5,
            ST_OUTPUT   = 'd6;

integer i, j;
genvar  a, b;
//---------------------------------------------------------------------
//   REG & WIRE
//---------------------------------------------------------------------
// SRAM MATRIX
reg   [9-1:0] MAT_A;
reg [256-1:0] MAT_DI;
wire[256-1:0] MAT_DO;
reg           MAT_WEB;
reg   [9-1:0] MAT_A_L;

// SRAM KERNEL
reg  [7-1:0] KER_A;   // 80 words
reg  [40-1:0] KER_DI;
wire [40-1:0] KER_DO;
reg          KER_WEB;
reg  [7-1:0] KER_A_IDX; 

// SRAM CONV
reg         [11-1:0] CONV_RA; // read addr
reg         [11-1:0] CONV_WA; // write addr
reg  signed [20-1:0] CONV_DI;
wire signed [20-1:0] CONV_DO;
reg           CONV_WEB;

// SRAM OUTPUT
reg         [11-1:0] OUT_RA; // read addr
reg         [11-1:0] OUT_WA; // write addr
reg  signed [20-1:0] OUT_DI;
wire signed [20-1:0] OUT_DO;
reg           OUT_WEB;

// SRAM OUTPUT REG
reg         [11-1:0] out_ra_n; // read addr
reg         [11-1:0] out_wa_n; // write addr
reg           out_web_n;

// INPUT
reg [7:0] data_in    [0:32-1];
reg [7:0] data_in_n  [0:32-1];
reg [3:0] mat_idx    [0:2-1];
reg [3:0] mat_idx_n  [0:2-1];
reg conv_mode;
reg conv_mode_n;
reg [4:0] mat_size, mat_size_n;
reg [1:0] mat_len, mat_len_n;

// FSM
reg [2:0] state, state_n;
reg [1:0] mp_state, mp_state_n;

// COUNTER
reg [9:0]    cnt,      cnt_n;
reg [5:0]    cnt_size, cnt_size_n;
reg [4:0]    cnt_addr, cnt_addr_n;
reg [2:0]    cnt_addr2, cnt_addr2_n;   // 0-4
reg [5:0]    cnt_row,   cnt_row_n;   // 0-35
reg [4:0]    cnt_out,   cnt_out_n;
reg [11-1:0] cnt_conv_raddr, cnt_conv_raddr_n;
reg [11-1:0] cnt_conv_waddr, cnt_conv_waddr_n;
reg          conv_web_reg, conv_web_reg_n;

// MUL AND ADDER
reg signed [7:0] pixel_0,    pixel_1,    pixel_2,    pixel_3,    pixel_4;
reg signed [7:0] pixel_0_n,  pixel_1_n,  pixel_2_n,  pixel_3_n,  pixel_4_n;

reg signed [7:0] kernel_0,   kernel_1,   kernel_2,   kernel_3,   kernel_4;
reg signed [7:0] kernel_0_n, kernel_1_n, kernel_2_n, kernel_3_n, kernel_4_n;

reg signed [15:0] mul_ans_0,    mul_ans_1,    mul_ans_2,    mul_ans_3,    mul_ans_4;
reg signed [15:0] mul_ans_0_n,  mul_ans_1_n,  mul_ans_2_n,  mul_ans_3_n,  mul_ans_4_n;

reg signed [20-1:0] sum_pixel, sum_pixel_n;

reg signed [20-1:0] add_result, add_result_n;

// MAX POOL
reg cnt_down, cnt_down_n;
reg signed [20-1:0] out_blk   [0:14-1];
reg signed [20-1:0] out_blk_n [0:14-1];

reg signed [20-1:0] cmp_blk   [0:2-1];
reg signed [20-1:0] cmp_blk_n [0:2-1];

reg signed [20-1:0] cmp_a;
reg signed [20-1:0] cmp_b;
reg signed [20-1:0] cmp_z_n;

// OUTPUT
reg out_valid_n, out_value_n;

//---------------------------------------------------------------------
//   DESIGN
//---------------------------------------------------------------------
// FSM
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        state    <= ST_IDLE;
        mp_state <= MP_IDLE;
    end
    else begin
        state    <= state_n;
        mp_state <= mp_state_n;
    end
end

always @(*) begin
    case(state)
        ST_IDLE:    begin
            if(in_valid)       state_n = ST_STORAGE;
            else if(in_valid2) state_n = ST_IDX;
            else               state_n = state;
        end 

        ST_STORAGE: begin
            if(&cnt_addr[3:0]) begin
                case (mat_len)
                    2'b00: begin 
                        if(MAT_A == {4'b1111, 2'b0, 3'b111}  && (&cnt_size[2:0])) state_n = ST_KERNEL;  // 487
                        else state_n = state;
                    end
                    2'b01: begin 
                        if(MAT_A == {4'b1111, 1'b0, 4'b1111} && (&cnt_size[3:0])) state_n = ST_KERNEL;  // 495
                        else state_n = state;
                    end
                    2'b10: begin
                        if(MAT_A == {4'b1111, 5'b11111}      && (&cnt_size[4:0])) state_n = ST_KERNEL;  // 511
                        else state_n = state;
                    end
                    default: state_n = state;
                endcase
            end
            else
                state_n = state;
        end

        ST_KERNEL: begin
            if(in_valid) state_n = ST_KERNEL;
            else         state_n = ST_IDLE;
        end

        ST_IDX: begin
            if(in_valid2)      state_n = ST_IDX;
            else if(conv_mode) state_n = ST_DECONV;
            else               state_n = ST_CONV;
        end

        ST_CONV: begin
            if(cnt == 1) begin
                case (mat_len)
                    2'b00: if(cnt_addr == 4)  state_n = ST_OUTPUT;     // offset
                           else state_n = state;
                    2'b01: if(cnt_addr == 12) state_n = ST_OUTPUT;
                           else state_n = state; 
                    2'b10: if(cnt_addr == 28) state_n = ST_OUTPUT; 
                           else state_n = state; 
                    default: state_n = state;
                endcase
            end
            else
                state_n = state;
        end

        ST_DECONV: begin
            if(cnt_size == 1) begin
                if(mat_len == 2'b00 && cnt_row == 12)
                    state_n = ST_OUTPUT;
                else if(mat_len == 2'b01 && cnt_row == 20)
                    state_n = ST_OUTPUT;
                else if(mat_len == 2'b10 && cnt_row == 36)
                    state_n = ST_OUTPUT;
                else
                    state_n = state;
            end
            else 
                state_n = state;
        end

        ST_OUTPUT: begin
            if(!out_valid)
                state_n = ST_IDLE;
            else
                state_n = state;
        end

        default: state_n = ST_IDLE;
    endcase
end

always @(*) begin
    case(mp_state)
        MP_IDLE: begin
            if(state == ST_CONV) begin
                if(mat_len == 2'b00)
                    if(cnt == 19)
                        mp_state_n = MP_UP;
                    else
                        mp_state_n = mp_state;
                else if(mat_len == 2'b01)
                    if(cnt == 51)
                        mp_state_n = MP_UP;
                    else
                        mp_state_n = mp_state;
                else begin
                    if(cnt == 115)
                        mp_state_n = MP_UP;
                    else
                        mp_state_n = mp_state;
                end
            end
            else begin
                mp_state_n = mp_state;
            end
        end

        MP_UP: begin
            if(cnt_size == 2)
                mp_state_n = MP_WAIT;
            else
                mp_state_n = mp_state;
        end

        MP_WAIT: begin
            if(mat_len == 2'b00)
                if(cnt == 19)
                    mp_state_n = MP_DOWN;
                else
                    mp_state_n = mp_state;
            else if(mat_len == 2'b01)
                if(cnt == 51)
                    mp_state_n = MP_DOWN;
                else
                    mp_state_n = mp_state;
            else begin
                if(cnt == 115)
                    mp_state_n = MP_DOWN;
                else
                    mp_state_n = mp_state;
            end
        end

        MP_DOWN: begin
            if(cnt == 3)
                mp_state_n = MP_IDLE;
            else
                mp_state_n = mp_state;
        end

        default: mp_state_n = mp_state;
    endcase
end
//----------------------------------------------//
// OUTPUT
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        out_valid <= 0;
        out_value <= 0;
    end
    else begin
        out_valid <= out_valid_n;
        out_value <= out_value_n;
    end
end
always @(*) begin
    if(state == ST_CONV) begin
        if(mp_state == MP_DOWN)
            if(OUT_RA[0] && !cnt_down)
                out_valid_n = 1'b1;
            else
                out_valid_n = out_valid;
        else
            out_valid_n = out_valid;
    end
    else if(state == ST_DECONV) begin
        if(cnt[2:0] == 6)
            out_valid_n = 1'b1;
        else
            out_valid_n = out_valid;
    end
    else if(state == ST_OUTPUT) begin
        if(conv_mode) begin
            if(mat_len == 2'b00 && OUT_RA == 408 && cnt_out == 19)
                out_valid_n = 0;
            else if(mat_len == 2'b01 && OUT_RA == 704 && cnt_out == 19)
                out_valid_n = 0;
            else if(mat_len == 2'b10 && OUT_RA == 0 && cnt_out == 19)
                out_valid_n = 0;
            else
                out_valid_n = out_valid;
        end
        else begin
            if(mat_len == 2'b00 && cnt == 42)
                out_valid_n = 0;
            else if(mat_len == 2'b01 && cnt == 114)
                out_valid_n = 0;
            else if(mat_len == 2'b10 && cnt == 258)
                out_valid_n = 0;
            else
                out_valid_n = out_valid;
        end
    end
    else begin
        out_valid_n = out_valid;
    end
end
always @(*) begin
    if(out_valid_n)
        out_value_n = out_blk[0][0];
    else
        out_value_n = 0;
end
//----------------------------------------------//
// input fetch
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        mat_size   <= 0;
        mat_len    <= 0;
        conv_mode  <= 0;
        mat_idx[0] <= 0;
        mat_idx[1] <= 0;
    end
    else begin
        mat_size   <= mat_size_n;
        mat_len    <= mat_len_n;
        conv_mode  <= conv_mode_n;
        mat_idx[0] <= mat_idx_n[0];
        mat_idx[1] <= mat_idx_n[1];
    end
end

always @(*) begin
    if(state == ST_IDLE && in_valid2) conv_mode_n = mode;
    else                              conv_mode_n = conv_mode;
end

always @(*) begin
    if(in_valid2) begin
        mat_idx_n[1] = matrix_idx;
        mat_idx_n[0] = mat_idx[1];
    end
    else begin
        mat_idx_n[1] = mat_idx[1];
        mat_idx_n[0] = mat_idx[0];
    end
end

always @(*) begin
    if(in_valid && state == ST_IDLE)
        case(matrix_size)
        2'd0:    mat_size_n = 5'b00111; // 7  = 8x8
        2'd1:    mat_size_n = 5'b01111; // 15 = 16x16
        2'd2:    mat_size_n = 5'b11111; // 31 = 32x32
        default: mat_size_n = 5'b00111;
        endcase
    else 
        mat_size_n = mat_size;
end
always @(*) begin
    if(in_valid && state == ST_IDLE)
        mat_len_n = matrix_size;
    else 
        mat_len_n = mat_len;
end
//----------------------------------------------//
generate
    for(a = 0; a < 32; a = a + 1) begin
        always @(posedge clk or negedge rst_n) begin
            if(~rst_n) begin
                data_in[a] <= 0;
            end
            else begin
                for(i = 0; i < 32; i = i + 1)
                    data_in[a] <= data_in_n[a];
            end
        end
    end
endgenerate


// LAST ELEMENT
always @(*) begin
    case (state)
        ST_IDLE: 
            if(in_valid)
                data_in_n[31] = matrix;
            else
                data_in_n[31] = data_in[31];

        ST_STORAGE:
            data_in_n[31] = matrix;

        ST_KERNEL:
            data_in_n[31] = matrix;

        default: data_in_n[31] = data_in[31];
    endcase
end
// REMAIN ELEMENT
generate
    for(a = 0; a < 31; a = a + 1) begin
        always @(*) begin
            data_in_n[a] = data_in[a + 1];
        end
    end
endgenerate

//----------------------------------------------//
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        OUT_RA  <= 0;
        OUT_WA  <= 0;
        OUT_WEB <= 1;
    end
    else begin
        OUT_RA  <= out_ra_n;
        OUT_WA  <= out_wa_n;
        OUT_WEB <= out_web_n;
    end
end

// SRAM DATA IN
always @(*) begin
    case(state)
        ST_IDLE:   MAT_DI = 256'b0;  // First element

        ST_STORAGE: begin
            case(mat_len)
                2'b00:
                    MAT_DI = {data_in[24], data_in[25], data_in[26], data_in[27], data_in[28], data_in[29], data_in[30], data_in[31], 192'b0};
                2'b01:
                    MAT_DI = {data_in[16], data_in[17], data_in[18], data_in[19], data_in[20], data_in[21], data_in[22], data_in[23],
                              data_in[24], data_in[25], data_in[26], data_in[27], data_in[28], data_in[29], data_in[30], data_in[31], 128'b0};
                2'b10:
                    MAT_DI = {data_in[0],  data_in[1], data_in[2],   data_in[3],  data_in[4],  data_in[5],  data_in[6],  data_in[7],
                              data_in[8],  data_in[9], data_in[10], data_in[11], data_in[12], data_in[13], data_in[14], data_in[15],
                              data_in[16], data_in[17], data_in[18], data_in[19], data_in[20], data_in[21], data_in[22], data_in[23],
                              data_in[24], data_in[25], data_in[26], data_in[27], data_in[28], data_in[29], data_in[30], data_in[31]};

                default: MAT_DI = 256'b0;
            endcase
        end

        default: MAT_DI = 256'b0;
    endcase
end

// SRAM ADDR IN
always @(*) begin
    case(state)
        ST_IDLE: MAT_A = 0;  // First element

        ST_STORAGE: begin
            case(mat_len)
                2'b00:
                    MAT_A = {cnt_addr[3:0], 2'b0, cnt[5:3]};
                2'b01: 
                    MAT_A = {cnt_addr[3:0], 1'b0, cnt[7:4]};
                2'b10:
                    MAT_A = {cnt_addr[3:0], cnt[9:5]};

                default: MAT_A = 9'b0;
            endcase
        end

        ST_KERNEL: MAT_A =  9'b0;

        ST_IDX:    MAT_A = {mat_idx[0], cnt_addr[4:0]}; // otherwise will not enough
        
        ST_CONV:   MAT_A = {mat_idx[0], cnt_addr[4:0]};

        ST_DECONV: MAT_A = {mat_idx[0], cnt_addr[4:0]};
        default: MAT_A = 9'b0;
    endcase
end

// SRAM WEN IN
always@(*)	begin
	case(state)
		ST_IDLE:	begin
            if(in_valid)    MAT_WEB = 1'b0;
            else            MAT_WEB = 1'b1;
        end	
		ST_STORAGE:	        MAT_WEB = 1'b0;
        ST_KERNEL:          MAT_WEB = 1'b1;
        default:            MAT_WEB = 1'b1;
	endcase
end
//----------------------------------------------//
// SRAM KERNEL IN
always @(*) begin
    case(state)
        ST_IDLE:    KER_DI = 40'b0;
        ST_STORAGE: KER_DI = 40'b0;
        ST_KERNEL:  KER_DI = {data_in[27], data_in[28], data_in[29], data_in[30], data_in[31]};

        default: KER_DI = 40'b0;
    endcase
end

// SRAM ADDR IN
always @(*) begin
    case(state)
        ST_IDLE:    KER_A = 6'b0;  // First element

        ST_STORAGE: KER_A = 6'b0;
        
        ST_KERNEL:  KER_A = cnt;

        ST_IDX:     KER_A = KER_A_IDX + cnt_addr2;
        
        ST_CONV:    KER_A = KER_A_IDX + cnt_addr2;

        ST_DECONV:    KER_A = KER_A_IDX + cnt_addr2;
        default: KER_A = 6'b0;
    endcase
end

always @(*) begin
    case(mat_idx[1])
        0:  KER_A_IDX = 0;
        1:  KER_A_IDX = 5;
        2:  KER_A_IDX = 10;
        3:  KER_A_IDX = 15;
        4:  KER_A_IDX = 20;
        5:  KER_A_IDX = 25;
        6:  KER_A_IDX = 30;
        7:  KER_A_IDX = 35;
        8:  KER_A_IDX = 40;
        9:  KER_A_IDX = 45;
        10: KER_A_IDX = 50;
        11: KER_A_IDX = 55;
        12: KER_A_IDX = 60;
        13: KER_A_IDX = 65;
        14: KER_A_IDX = 70;
        15: KER_A_IDX = 75;
        default: KER_A_IDX = 0;
    endcase 
end
// SRAM WEN IN
always@(*)	begin
	case(state)
		ST_IDLE:            KER_WEB = 1'b1;
		ST_STORAGE:	        KER_WEB = 1'b1;
        ST_KERNEL:          KER_WEB = 1'b0;
        default:            KER_WEB = 1'b1;
	endcase
end
//----------------------------------------------//
// CONV SRAM
always @(*) begin
    CONV_RA  = cnt_conv_raddr;
    CONV_WA  = cnt_conv_waddr;
    CONV_WEB = conv_web_reg;
    CONV_DI  = add_result;
end

//----------------------------------------------//
// CONV DATA
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        pixel_0    <= 0;
        pixel_1    <= 0;
        pixel_2    <= 0;
        pixel_3    <= 0;
        pixel_4    <= 0;
        kernel_0   <= 0;
        kernel_1   <= 0;
        kernel_2   <= 0;
        kernel_3   <= 0;
        kernel_4   <= 0;
        mul_ans_0  <= 0;
        mul_ans_1  <= 0;
        mul_ans_2  <= 0;
        mul_ans_3  <= 0;
        mul_ans_4  <= 0;
        sum_pixel  <= 0;
        add_result <= 0;
    end
    else begin
        pixel_0    <= pixel_0_n;
        pixel_1    <= pixel_1_n;
        pixel_2    <= pixel_2_n;
        pixel_3    <= pixel_3_n;
        pixel_4    <= pixel_4_n;
        kernel_0   <= kernel_0_n;
        kernel_1   <= kernel_1_n;
        kernel_2   <= kernel_2_n;
        kernel_3   <= kernel_3_n;
        kernel_4   <= kernel_4_n;
        mul_ans_0  <= mul_ans_0_n;
        mul_ans_1  <= mul_ans_1_n;
        mul_ans_2  <= mul_ans_2_n;
        mul_ans_3  <= mul_ans_3_n;
        mul_ans_4  <= mul_ans_4_n;
        sum_pixel  <= sum_pixel_n;
        add_result <= add_result_n;
    end
end

always @(*) begin
    case(state)
        ST_CONV: begin
            case(mat_len)
                2'b00: begin
                    if(cnt <= 4 && cnt != 0) sum_pixel_n = 0;
                    else                     sum_pixel_n = CONV_DO;
                end 

                2'b01: begin
                    if(cnt <= 12 && cnt != 0) sum_pixel_n = 0;
                    else                      sum_pixel_n = CONV_DO;
                end

                2'b10: begin
                    if(cnt <= 28 && cnt != 0) sum_pixel_n = 0;
                    else                      sum_pixel_n = CONV_DO;
                end

                default: sum_pixel_n = 0;
            endcase
        end

        ST_DECONV: begin
            if(mat_len == 2'b00) begin
                if(cnt_row == 1)
                    if(cnt <= 12)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
                else
                    if(cnt <= 12 && cnt != 0)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
            end
            else if(mat_len == 2'b01) begin
                if(cnt_row == 1)
                    if(cnt <= 20)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
                else
                    if(cnt <= 20 && cnt != 0)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
            end
            else begin
                if(cnt_row == 1)
                    if(cnt <= 36)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
                else
                    if(cnt <= 36 && cnt != 0)
                        sum_pixel_n = 0;
                    else
                        sum_pixel_n = CONV_DO;
            end
        end

        default: sum_pixel_n = 0;
    endcase
end
// MUL
always @(*) begin
    if(conv_mode) begin
        if(mat_len == 2'b00 && cnt_size >= 8) begin
            pixel_4_n = 0;
        end
        else if(mat_len == 2'b01 && cnt_size >= 16) begin
            pixel_4_n = 0;
        end
        else if(mat_len == 2'b10 && cnt_size >= 32) begin
            pixel_4_n = 0;
        end
        else begin
            case (cnt_size)
                0:  pixel_4_n = MAT_DO[255:248];
                1:  pixel_4_n = MAT_DO[247:240];
                2:  pixel_4_n = MAT_DO[239:232];
                3:  pixel_4_n = MAT_DO[231:224];
                4:  pixel_4_n = MAT_DO[223:216];
                5:  pixel_4_n = MAT_DO[215:208];
                6:  pixel_4_n = MAT_DO[207:200];
                7:  pixel_4_n = MAT_DO[199:192];
                8:  pixel_4_n = MAT_DO[191:184];
                9:  pixel_4_n = MAT_DO[183:176];
                10: pixel_4_n = MAT_DO[175:168];
                11: pixel_4_n = MAT_DO[167:160];
                12: pixel_4_n = MAT_DO[159:152];
                13: pixel_4_n = MAT_DO[151:144];
                14: pixel_4_n = MAT_DO[143:136];
                15: pixel_4_n = MAT_DO[135:128];
                16: pixel_4_n = MAT_DO[127:120];
                17: pixel_4_n = MAT_DO[119:112];
                18: pixel_4_n = MAT_DO[111:104];
                19: pixel_4_n = MAT_DO[103:96];
                20: pixel_4_n = MAT_DO[95:88];
                21: pixel_4_n = MAT_DO[87:80];
                22: pixel_4_n = MAT_DO[79:72];
                23: pixel_4_n = MAT_DO[71:64];
                24: pixel_4_n = MAT_DO[63:56];
                25: pixel_4_n = MAT_DO[55:48];
                26: pixel_4_n = MAT_DO[47:40];
                27: pixel_4_n = MAT_DO[39:32];
                28: pixel_4_n = MAT_DO[31:24];
                29: pixel_4_n = MAT_DO[23:16];
                30: pixel_4_n = MAT_DO[15:8];
                31: pixel_4_n = MAT_DO[7:0];
                default: pixel_4_n = 0;
            endcase
        end
    end
    else begin
        case (cnt_size)
            0:  pixel_4_n = MAT_DO[223:216];
            1:  pixel_4_n = MAT_DO[215:208];
            2:  pixel_4_n = MAT_DO[207:200];
            3:  pixel_4_n = MAT_DO[199:192];
            4:  pixel_4_n = MAT_DO[191:184];
            5:  pixel_4_n = MAT_DO[183:176];
            6:  pixel_4_n = MAT_DO[175:168];
            7:  pixel_4_n = MAT_DO[167:160];
            8:  pixel_4_n = MAT_DO[159:152];
            9:  pixel_4_n = MAT_DO[151:144];
            10: pixel_4_n = MAT_DO[143:136];
            11: pixel_4_n = MAT_DO[135:128];
            12: pixel_4_n = MAT_DO[127:120];
            13: pixel_4_n = MAT_DO[119:112];
            14: pixel_4_n = MAT_DO[111:104];
            15: pixel_4_n = MAT_DO[103:96];
            16: pixel_4_n = MAT_DO[95:88];
            17: pixel_4_n = MAT_DO[87:80];
            18: pixel_4_n = MAT_DO[79:72];
            19: pixel_4_n = MAT_DO[71:64];
            20: pixel_4_n = MAT_DO[63:56];
            21: pixel_4_n = MAT_DO[55:48];
            22: pixel_4_n = MAT_DO[47:40];
            23: pixel_4_n = MAT_DO[39:32];
            24: pixel_4_n = MAT_DO[31:24];
            25: pixel_4_n = MAT_DO[23:16];
            26: pixel_4_n = MAT_DO[15:8];
            27: pixel_4_n = MAT_DO[7:0];
            default: pixel_4_n = 0;
        endcase
    end
end

always @(*) begin
    if(conv_mode) begin
        if(cnt_size == 0) begin
            pixel_3_n = 0;
            pixel_2_n = 0;
            pixel_1_n = 0;
            pixel_0_n = 0;
        end
        else begin
            pixel_3_n = pixel_4;
            pixel_2_n = pixel_3;
            pixel_1_n = pixel_2;
            pixel_0_n = pixel_1;
        end
    end
    else begin
        if(cnt_size == 0) begin
            pixel_3_n = MAT_DO[231:224];
            pixel_2_n = MAT_DO[239:232];
            pixel_1_n = MAT_DO[247:240];
            pixel_0_n = MAT_DO[255:248];
        end
        else begin
            pixel_3_n = pixel_4;
            pixel_2_n = pixel_3;
            pixel_1_n = pixel_2;
            pixel_0_n = pixel_1;
        end
    end
end

always @(*) begin
    if(conv_mode) begin
        kernel_4_n = KER_DO[39:32]; // A
        kernel_3_n = KER_DO[31:24]; // B
        kernel_2_n = KER_DO[23:16]; // C
        kernel_1_n = KER_DO[15:8];  // D
        kernel_0_n = KER_DO[7:0];   // E
    end
    else begin
        kernel_4_n = KER_DO[7:0];   // E
        kernel_3_n = KER_DO[15:8];  // D
        kernel_2_n = KER_DO[23:16]; // C
        kernel_1_n = KER_DO[31:24]; // B
        kernel_0_n = KER_DO[39:32]; // A
    end
    
end

always @(*) begin
    mul_ans_4_n = pixel_4 * kernel_4;
    mul_ans_3_n = pixel_3 * kernel_3;
    mul_ans_2_n = pixel_2 * kernel_2;
    mul_ans_1_n = pixel_1 * kernel_1;
    mul_ans_0_n = pixel_0 * kernel_0;
end

always @(*) begin
    add_result_n = mul_ans_0 + mul_ans_1 + mul_ans_2 + mul_ans_3 + mul_ans_4 + sum_pixel;
end
//----------------------------------------------//
// MAX POOL
generate
    for(a = 0; a < 14; a = a + 1) begin
        always @(posedge clk or negedge rst_n) begin
            if(~rst_n) begin
                out_blk[a] <= 0;
            end
            else begin
                out_blk[a] <= out_blk_n[a];
            end
        end
    end
endgenerate
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        cmp_blk[0] <= 0;
        cmp_blk[1] <= 0;
        cnt_down   <= 0;
    end
    else begin
        cmp_blk[0] <= cmp_blk_n[0];
        cmp_blk[1] <= cmp_blk_n[1];
        cnt_down   <= cnt_down_n;
    end
end
always @(*) begin
    if(mp_state == MP_DOWN)
        if(!cnt_down)
            cmp_blk_n[0] = OUT_DO;
        else
            cmp_blk_n[0] = CONV_DI;
    else
        cmp_blk_n[0] = CONV_DI;
end

always @(*) begin
    cmp_blk_n[1] = cmp_z_n;
end
always @(*) begin
    if(mp_state == MP_DOWN) begin
        if(!cnt_down) begin
            cmp_a = cmp_blk[0];
            cmp_b = CONV_DI;
        end
        else begin
            cmp_a = cmp_blk[0];
            cmp_b = cmp_blk[1];
        end
    end
    else begin
        cmp_a = cmp_blk[0];
        cmp_b = CONV_DI;
    end
end
always @(*) begin
    if(cmp_a > cmp_b)
        cmp_z_n = cmp_a;
    else
        cmp_z_n = cmp_b;
end
always @(*) begin
    if(mp_state == MP_DOWN)
        cnt_down_n = ~cnt_down;
    else
        cnt_down_n = 0;
end
//----------------------------------------------//
// OUTPUT BLOCK
always @(*) begin
    if(conv_mode) begin
        if(cnt_out == 18)
            out_blk_n[0] = OUT_DO;
        else if(out_valid_n)
            out_blk_n[0] = {1'b0, out_blk[0][19:1]};
        else if(cnt[2:0] == 5)
            out_blk_n[0] = OUT_DO;
        else
            out_blk_n[0] = 0;
    end
    else begin
        if(mp_state == MP_DOWN && cnt_down && OUT_RA == 1)
            out_blk_n[0] = cmp_z_n;

        else if(cnt_out == 18)
            out_blk_n[0] = out_blk[1];

        else if(out_valid_n)
            out_blk_n[0] = {1'b0, out_blk[0][19:1]};
        else
            out_blk_n[0] = out_blk[0];
    end
end
generate
    for(a = 1; a <= 8; a = a + 1) begin
        always @(*) begin
            if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == (a+1)) begin
                out_blk_n[a] = cmp_z_n;
            end
            else if(cnt_out == 18)
                out_blk_n[a] = out_blk[a+1];
            else
                out_blk_n[a] = out_blk[a];
        end
    end
endgenerate
// 9
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 10)
        out_blk_n[9] = cmp_z_n;
    else if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 11)
        out_blk_n[9] = cmp_z_n;
    else if(cnt_out == 18)
        out_blk_n[9] = out_blk[10];
    else
        out_blk_n[9] = out_blk[9];
end
// 10
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 12)
        out_blk_n[10] = cmp_z_n;
    else if(cnt_out == 18)
        out_blk_n[10] = out_blk[11];
    else
        out_blk_n[10] = out_blk[10];
end
// 11
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 13)
        out_blk_n[11] = cmp_z_n;
    else if(cnt_out == 18)
        out_blk_n[11] = out_blk[12];
    else
        out_blk_n[11] = out_blk[11];
end
// 12
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 14)
        out_blk_n[12] = cmp_z_n;
    else if(cnt_out == 18)
        out_blk_n[12] = 0;
    else
        out_blk_n[12] = out_blk[12];
end
/*
// blk 13
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[3:0] == 14) begin
        out_blk_n[13] = cmp_z_n;
    end
    else if(cnt_out == 18)
        out_blk_n[13] = 0;
    else
        out_blk_n[13] = out_blk[13];
end
*/
/*
always @(*) begin
    if(mp_state == MP_DOWN && cnt_down && OUT_RA[2:0] == 2) begin
        out_blk_n[1] = cmp_z_n;
    end
    else if(cnt_out == 18)
        out_blk_n[1] = out_blk[2];
    else
        out_blk_n[1] = out_blk[1];
end
*/
//----------------------------------------------//
// SRAM OUT (MAX POOL)
always @(*) begin
    if(state == ST_CONV) begin
        if(mat_len == 2'b00) begin
            if(mp_state == MP_IDLE && cnt == 19)
                out_web_n = 0;
            else if(mp_state == MP_UP)
                out_web_n = ~OUT_WEB;
            else
                out_web_n = 1;
        end 
        else if(mat_len == 2'b01) begin
            if(mp_state == MP_IDLE && cnt == 51)
                out_web_n = 0;
            else if(mp_state == MP_UP)
                out_web_n = ~OUT_WEB;
            else
                out_web_n = 1;
        end 
        else begin
            if(mp_state == MP_IDLE && cnt == 115)
                out_web_n = 0;
            else if(mp_state == MP_UP)
                out_web_n = ~OUT_WEB;
            else
                out_web_n = 1;
        end   
    end
    else if(state == ST_DECONV) begin
        out_web_n = conv_web_reg_n;
    end
    else begin
        out_web_n = 1;
    end
end
always @(*) begin
    if(state == ST_CONV) begin
        if(mp_state == MP_UP)
            if(OUT_WEB == 1)
                out_wa_n = OUT_WA + 1;
            else
                out_wa_n = OUT_WA;
        else
            out_wa_n = 0;
    end
    else if(state == ST_DECONV) begin
        out_wa_n = cnt_conv_waddr_n;
    end
    else begin
        out_wa_n = 0;
    end
end
always @(*) begin
    if(conv_mode) begin
        if(out_valid) begin
            if(cnt_out == 16) begin
                if(mat_len == 2'b00) begin
                    case(OUT_RA)
                        11: out_ra_n = 36;
                        47: out_ra_n = 72;
                        83: out_ra_n = 108;
                        119: out_ra_n = 144;
                        155: out_ra_n = 180;
                        191: out_ra_n = 216;
                        227: out_ra_n = 252;
                        263: out_ra_n = 288;
                        299: out_ra_n = 324;
                        335: out_ra_n = 360;
                        371: out_ra_n = 396;
                        default: out_ra_n = OUT_RA + 1;
                    endcase
                end
                else if(mat_len == 2'b01) begin
                    case(OUT_RA)
                        19: out_ra_n = 36;
                        55: out_ra_n = 72;
                        91: out_ra_n = 108;
                        127: out_ra_n = 144;
                        163: out_ra_n = 180;
                        199: out_ra_n = 216;
                        235: out_ra_n = 252;
                        271: out_ra_n = 288;
                        307: out_ra_n = 324;
                        343: out_ra_n = 360;
                        379: out_ra_n = 396;
                        415: out_ra_n = 432;
                        451: out_ra_n = 468;
                        487: out_ra_n = 504;
                        523: out_ra_n = 540;
                        559: out_ra_n = 576;
                        595: out_ra_n = 612;
                        631: out_ra_n = 648;
                        667: out_ra_n = 684;
                        default: out_ra_n = OUT_RA + 1;
                    endcase
                end
                else if(OUT_RA == 1295)
                    out_ra_n = 0;
                else
                    out_ra_n = OUT_RA + 1;
            end
            else
                out_ra_n = OUT_RA;
            end
        else if(OUT_WEB == 0)
            out_ra_n = 0;
        else
            out_ra_n = 15;
    end
    else begin
        if(mp_state == MP_IDLE || mp_state == MP_UP)
            out_ra_n = 15;
        else if(mp_state == MP_DOWN)
            if(!cnt_down)
                out_ra_n = OUT_RA + 1;
            else
                out_ra_n = OUT_RA;
        else 
            out_ra_n = 0;
    end
end
always @(*) begin
    if(conv_mode) begin
        OUT_DI = CONV_DI;
    end
    else begin
        OUT_DI = cmp_z_n;
    end
end
//----------------------------------------------//
// FLAG
reg done_layer_0, done_layer_0_n;
reg done_layer_1, done_layer_1_n;
reg done_layer_2, done_layer_2_n;
reg done_layer_3, done_layer_3_n;
reg done_layer_0_b, done_layer_0_b_n;
reg done_layer_1_b, done_layer_1_b_n;
reg done_layer_2_b, done_layer_2_b_n;
reg done_layer_3_b, done_layer_3_b_n;

reg done_layer_4, done_layer_4_n;

always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        done_layer_0 <= 0;
        done_layer_1 <= 0;
        done_layer_2 <= 0;
        done_layer_3 <= 0;
        done_layer_4 <= 0;
        done_layer_0_b <= 0;
        done_layer_1_b <= 0;
        done_layer_2_b <= 0;
        done_layer_3_b <= 0;
    end
    else begin
        done_layer_0 <= done_layer_0_n;
        done_layer_1 <= done_layer_1_n;
        done_layer_2 <= done_layer_2_n;
        done_layer_3 <= done_layer_3_n;
        done_layer_4 <= done_layer_4_n;
        done_layer_0_b <= done_layer_0_b_n;
        done_layer_1_b <= done_layer_1_b_n;
        done_layer_2_b <= done_layer_2_b_n;
        done_layer_3_b <= done_layer_3_b_n;
    end
end
// FRONT FLAG
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 0 && cnt == 10)
            done_layer_0_n = 1'b1;
        else
            done_layer_0_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 0 && cnt == 18)
            done_layer_0_n = 1'b1;
        else
            done_layer_0_n = 1'b0;
    end 
    else begin
        if(cnt_row == 0 && cnt == 34)
            done_layer_0_n = 1'b1;
        else
            done_layer_0_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 1 && cnt == 22)
            done_layer_1_n = 1'b1;
        else
            done_layer_1_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 1 && cnt == 38)
            done_layer_1_n = 1'b1;
        else
            done_layer_1_n = 1'b0;
    end 
    else begin
        if(cnt_row == 1 && cnt == 70)
            done_layer_1_n = 1'b1;
        else
            done_layer_1_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 2 && cnt == 34)
            done_layer_2_n = 1'b1;
        else
            done_layer_2_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
         if(cnt_row == 2 && cnt == 58)
            done_layer_2_n = 1'b1;
        else
            done_layer_2_n = 1'b0;
    end 
    else begin
        if(cnt_row == 2 && cnt == 106)
            done_layer_2_n = 1'b1;
        else
            done_layer_2_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 3 && cnt == 46)
            done_layer_3_n = 1'b1;
        else
            done_layer_3_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 3 && cnt == 78)
            done_layer_3_n = 1'b1;
        else
            done_layer_3_n = 1'b0;
    end 
    else begin
        if(cnt_row == 3 && cnt == 142)
            done_layer_3_n = 1'b1;
        else
            done_layer_3_n = 1'b0;
    end  
end
// BACK FLAG
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 11 && cnt == 10)
            done_layer_0_b_n = 1'b1;
        else
            done_layer_0_b_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 19 && cnt == 18)
            done_layer_0_b_n = 1'b1;
        else
            done_layer_0_b_n = 1'b0;
    end 
    else begin
        if(cnt_row == 35 && cnt == 34)
            done_layer_0_b_n = 1'b1;
        else
            done_layer_0_b_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 10 && cnt == 22)
            done_layer_1_b_n = 1'b1;
        else
            done_layer_1_b_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 18 && cnt == 38)
            done_layer_1_b_n = 1'b1;
        else
            done_layer_1_b_n = 1'b0;
    end 
    else begin
        if(cnt_row == 34 && cnt == 70)
            done_layer_1_b_n = 1'b1;
        else
            done_layer_1_b_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 9 && cnt == 34)
            done_layer_2_b_n = 1'b1;
        else
            done_layer_2_b_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
         if(cnt_row == 17 && cnt == 58)
            done_layer_2_b_n = 1'b1;
        else
            done_layer_2_b_n = 1'b0;
    end 
    else begin
        if(cnt_row == 33 && cnt == 106)
            done_layer_2_b_n = 1'b1;
        else
            done_layer_2_b_n = 1'b0;
    end  
end
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt_row == 8 && cnt == 46)
            done_layer_3_b_n = 1'b1;
        else
            done_layer_3_b_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt_row == 16 && cnt == 78)
            done_layer_3_b_n = 1'b1;
        else
            done_layer_3_b_n = 1'b0;
    end 
    else begin
        if(cnt_row == 32 && cnt == 142)
            done_layer_3_b_n = 1'b1;
        else
            done_layer_3_b_n = 1'b0;
    end  
end
// GENERAL 25 times
always @(*) begin
    if(mat_len == 2'b00) begin
        if(cnt == 58)
            done_layer_4_n = 1'b1;
        else
            done_layer_4_n = 1'b0;
    end 
    else if(mat_len == 2'b01) begin
        if(cnt == 98)
            done_layer_4_n = 1'b1;
        else
            done_layer_4_n = 1'b0;
    end 
    else begin
        if(cnt == 178)
            done_layer_4_n = 1'b1;
        else
            done_layer_4_n = 1'b0;
    end  
end
// COUNTER
always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
        cnt       <= 0;
        cnt_addr  <= 0;
        cnt_addr2 <= 0;
        cnt_size  <= 0;
        cnt_row   <= 0;
        cnt_out   <= 0;
        cnt_conv_raddr <= 0;
        cnt_conv_waddr <= 0;
        
        conv_web_reg  <= 1;
        
    end
    else begin
        cnt       <= cnt_n;
        cnt_addr  <= cnt_addr_n;
        cnt_addr2 <= cnt_addr2_n;
        cnt_size  <= cnt_size_n;
        cnt_row   <= cnt_row_n;
        cnt_out   <= cnt_out_n;
        cnt_conv_raddr <= cnt_conv_raddr_n;
        cnt_conv_waddr <= cnt_conv_waddr_n;
        conv_web_reg  <= conv_web_reg_n;
    end
end

always @(*) begin
    if(out_valid)
        if(cnt_out == 19)
            cnt_out_n = 0;
        else
            cnt_out_n = cnt_out + 1;
    else
        cnt_out_n = 0;
end

always @(*) begin
    case (state)
        ST_IDLE: 
            cnt_n = 0;

        ST_STORAGE:
            cnt_n = cnt + 1'b1;

        ST_KERNEL: begin
            if(cnt_size[2]) // if cnt_size = 4
                cnt_n = cnt + 1'b1;
            else
                cnt_n = cnt;
        end

        ST_CONV: begin
            case(mat_len)
                2'b00:
                    if(cnt[4] && !cnt[3] && !cnt[2] && cnt[1] && cnt[0]) cnt_n = 0;    // if cnt = 19
                    else cnt_n = cnt + 1'b1;
                2'b01:
                    if(cnt[5] && cnt[4] && cnt[3] && !cnt[2] && cnt[1] && cnt[0]) cnt_n = 0;    // if cnt = 59
                    else cnt_n = cnt + 1'b1;
                2'b10:
                    if(cnt[7] && !cnt[6] && !cnt[5] && !cnt[4] && cnt[3] && !cnt[2] && cnt[1] && cnt[0]) cnt_n = 0;    // if cnt = 139
                    else cnt_n = cnt + 1'b1;
                
                default: cnt_n = cnt + 1'b1;
			endcase
        end

        ST_DECONV: begin
            if(done_layer_0)
                cnt_n = 0;
            else if(done_layer_1 || done_layer_1_b)
                cnt_n = 0;
            else if(done_layer_2 || done_layer_2_b)
                cnt_n = 0;
            else if(done_layer_3 || done_layer_3_b)
                cnt_n = 0;
            else if(done_layer_4)
                cnt_n = 0;
            else
                cnt_n = cnt + 1'b1;
        end

        ST_OUTPUT: begin
            cnt_n = cnt + 1;    
        end

        default: cnt_n = 0;
    endcase
end

always @(*) begin
    case(state)
		ST_IDLE: 
			cnt_addr_n = 0;
	
		ST_STORAGE:	begin
			case(mat_len)
                2'b00: begin
                    if(&cnt[5:0]) cnt_addr_n = cnt_addr + 1;    // if cnt = 8x8 = 64
                    else          cnt_addr_n = cnt_addr;
                end
                2'b01: begin
                    if(&cnt[7:0]) cnt_addr_n = cnt_addr + 1;    // if cnt = 16*16 = 256
                    else          cnt_addr_n = cnt_addr;
                end
                2'b10: begin
                    if(&cnt[9:0]) cnt_addr_n = cnt_addr + 1;    // if cnt = 32*32 = 1024
                    else          cnt_addr_n = cnt_addr;
                end
                
                default:          cnt_addr_n = cnt_addr;
			endcase
		end

        ST_CONV: begin  // matrix addr
			case(mat_len)
                2'b00: begin
                    if(cnt[4] && !cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr_n = cnt_addr - 3;    // if cnt = 18
                    else if(cnt_size[1] && !cnt_size[0])
                        cnt_addr_n = cnt_addr + 1;    // if cnt size = 2
                    else
                        cnt_addr_n = cnt_addr;
                end

                2'b01: begin
                    if(cnt[5] && cnt[4] && cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr_n = cnt_addr - 3;    // if cnt = 58
                    else if(cnt_size[3] && !cnt_size[2] && cnt_size[1] && !cnt_size[0]) 
                        cnt_addr_n = cnt_addr + 1;    // if cnt size = 10
                    else
                        cnt_addr_n = cnt_addr;
                end

                2'b10: begin
                    if(cnt[7] && !cnt[6] && !cnt[5] && !cnt[4] && cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr_n = cnt_addr - 3;    // if cnt = 138
                    else if(cnt_size[4] && cnt_size[3] && !cnt_size[2] && cnt_size[1] && !cnt_size[0]) 
                        cnt_addr_n = cnt_addr + 1;    // if cnt size = 26
                    else
                        cnt_addr_n = cnt_addr;
                end
                default: cnt_addr_n = cnt_addr;
			endcase
		end
        
        ST_DECONV: begin
            case(mat_len)
                2'b00: begin
                    // CHANGE
                    if(done_layer_4_n)
                        cnt_addr_n = cnt_addr - 3;
                    else if(done_layer_0_n || done_layer_1_n || done_layer_2_n || done_layer_3_n)
                        cnt_addr_n = 0;
                    else if(done_layer_1_b_n) 
                        cnt_addr_n = 7;
                    else if(done_layer_2_b_n) // row 9
                        cnt_addr_n = 6;
                    else if(done_layer_3_b_n) // row 8
                        cnt_addr_n = 5;
                    // NEXT ROW
                    else if(cnt_size == 10)
                        cnt_addr_n = cnt_addr + 1;
                    else
                        cnt_addr_n = cnt_addr;
                end 

                2'b01: begin
                    // CHANGE
                    if(done_layer_4_n)
                        cnt_addr_n = cnt_addr - 3;
                    else if(done_layer_0_n || done_layer_1_n || done_layer_2_n || done_layer_3_n)
                        cnt_addr_n = 0;
                    else if(done_layer_1_b_n) 
                        cnt_addr_n = 15;
                    else if(done_layer_2_b_n) // row 17
                        cnt_addr_n = 14;
                    else if(done_layer_3_b_n) // row 16
                        cnt_addr_n = 13;
                    // NEXT ROW
                    else if(cnt_size == 18)
                        cnt_addr_n = cnt_addr + 1;
                    else
                        cnt_addr_n = cnt_addr;
                end

                2'b10: begin
                    // CHANGE
                    if(done_layer_4_n)
                        cnt_addr_n = cnt_addr - 3;
                    else if(done_layer_0_n || done_layer_1_n || done_layer_2_n || done_layer_3_n)
                        cnt_addr_n = 0;
                    else if(done_layer_1_b_n) 
                        cnt_addr_n = 31;
                    else if(done_layer_2_b_n) // row 33
                        cnt_addr_n = 30;
                    else if(done_layer_3_b_n) // row 32
                        cnt_addr_n = 29;
                    // NEXT ROW
                    else if(cnt_size == 34)
                        cnt_addr_n = cnt_addr + 1;
                    else
                        cnt_addr_n = cnt_addr;
                end
                default: cnt_addr_n = cnt_addr;
			endcase
        end

        default: cnt_addr_n = 0;
    endcase
end

always @(*) begin
    case(state)
        ST_CONV: begin  // kernel addr
			case(mat_len)
                2'b00: begin
                    if(cnt[4] && !cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr2_n = 0;    // if cnt = 18
                    else if(cnt_size[1] && !cnt_size[0])
                        cnt_addr2_n = cnt_addr2 + 1;    // if cnt size = 2
                    else
                        cnt_addr2_n = cnt_addr2;
                end

                2'b01: begin
                    if(cnt[5] && cnt[4] && cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr2_n = 0;    // if cnt = 58
                    else if(cnt_size[3] && !cnt_size[2] && cnt_size[1] && !cnt_size[0]) 
                        cnt_addr2_n = cnt_addr2 + 1;    // if cnt size = 10
                    else
                        cnt_addr2_n = cnt_addr2;
                end

                2'b10: begin
                    if(cnt[7] && !cnt[6] && !cnt[5] && !cnt[4] && cnt[3] && !cnt[2] && cnt[1] && !cnt[0])
                        cnt_addr2_n = 0;    // if cnt = 138
                    else if(cnt_size[4] && cnt_size[3] && !cnt_size[2] && cnt_size[1] && !cnt_size[0]) 
                        cnt_addr2_n = cnt_addr2 + 1;    // if cnt size = 26
                    else
                        cnt_addr2_n = cnt_addr2;
                end
                default: cnt_addr2_n = cnt_addr2;
			endcase
		end
        
        ST_DECONV: begin
            // CHANGE
            if(done_layer_3_n || done_layer_4_n || done_layer_1_b_n || done_layer_2_b_n || done_layer_3_b_n)
                cnt_addr2_n = 4;
            else if(done_layer_0_n)
                cnt_addr2_n = 1;
            else if(done_layer_1_n) 
                cnt_addr2_n = 2;
            else if(done_layer_2_n)
                cnt_addr2_n = 3;
            else begin
                if(mat_len == 2'b00)
                    if(cnt_size == 10)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                else if(mat_len == 2'b01)
                    if(cnt_size == 18)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                else begin
                    if(cnt_size == 34)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                end
            end
            /*
            case(mat_len)
                2'b00: begin
                    // CHANGE
                    if(done_layer_3 || done_layer_4 || done_layer_1_b || done_layer_2_b || done_layer_3_b)
                        cnt_addr2_n = 4;
                    else if(done_layer_0)
                        cnt_addr2_n = 1;
                    else if(done_layer_1) 
                        cnt_addr2_n = 2;
                    else if(done_layer_2)
                        cnt_addr2_n = 3;

                    // NEXT ROW
                    else if(cnt_size == 10)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                end 

                2'b01: begin
                    // CHANGE
                    if(done_layer_3 || done_layer_4 || done_layer_1_b || done_layer_2_b || done_layer_3_b)
                        cnt_addr2_n = 4;
                    else if(done_layer_0)
                        cnt_addr2_n = 1;
                    else if(done_layer_1) 
                        cnt_addr2_n = 2;
                    else if(done_layer_2)
                        cnt_addr2_n = 3;

                    // NEXT ROW
                    else if(cnt_size == 18)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                end

                2'b10: begin
                    // CHANGE
                    if(done_layer_3 || done_layer_4 || done_layer_1_b || done_layer_2_b || done_layer_3_b)
                        cnt_addr2_n = 4;
                    else if(done_layer_0)
                        cnt_addr2_n = 1;
                    else if(done_layer_1) 
                        cnt_addr2_n = 2;
                    else if(done_layer_2)
                        cnt_addr2_n = 3;

                    // NEXT ROW
                    else if(cnt_size == 34)
                        cnt_addr2_n = cnt_addr2 - 1;
                    else
                        cnt_addr2_n = cnt_addr2;
                end
                default: cnt_addr2_n = cnt_addr2;
			endcase
            */
        end
        default: cnt_addr2_n = 0;
    endcase
end

always @(*) begin
    case(state)
        ST_CONV: begin  // conv addr
            if((cnt[4] && !cnt[3] && !cnt[2] && cnt[1] && cnt[0] && mat_len == 0) ||                                // cnt = 19
               (cnt[5] && cnt[4] && cnt[3] && !cnt[2] && cnt[1] && cnt[0] && mat_len[0]) ||                         // cnt = 59
               (cnt[7] && !cnt[6] && !cnt[5] && !cnt[4] && cnt[3] && !cnt[2] && cnt[1] && cnt[0] && mat_len[1]))    // cnt = 139
            begin
                case (cnt_addr) // MAT_A offset
                    1:  cnt_conv_raddr_n = 36;
                    2:  cnt_conv_raddr_n = 72;
                    3:  cnt_conv_raddr_n = 108;
                    4:  cnt_conv_raddr_n = 144;
                    5:  cnt_conv_raddr_n = 180;
                    6:  cnt_conv_raddr_n = 216;
                    7:  cnt_conv_raddr_n = 252;
                    8:  cnt_conv_raddr_n = 288;
                    9:  cnt_conv_raddr_n = 324;
                    10: cnt_conv_raddr_n = 360;
                    11: cnt_conv_raddr_n = 396;
                    12: cnt_conv_raddr_n = 432;
                    13: cnt_conv_raddr_n = 468;
                    14: cnt_conv_raddr_n = 504;
                    15: cnt_conv_raddr_n = 540;
                    16: cnt_conv_raddr_n = 576;
                    17: cnt_conv_raddr_n = 612;
                    18: cnt_conv_raddr_n = 648;
                    19: cnt_conv_raddr_n = 684;
                    20: cnt_conv_raddr_n = 720;
                    21: cnt_conv_raddr_n = 756;
                    22: cnt_conv_raddr_n = 792;
                    23: cnt_conv_raddr_n = 828;
                    24: cnt_conv_raddr_n = 864;
                    25: cnt_conv_raddr_n = 900;
                    26: cnt_conv_raddr_n = 936;
                    27: cnt_conv_raddr_n = 972;
                    default: cnt_conv_raddr_n = 0;
                endcase
            end
            else begin
                case(mat_len)
                    2'b00:  begin
                        cnt_conv_raddr_n = cnt_conv_raddr + 1;
                        if(cnt_conv_raddr == 3)   cnt_conv_raddr_n = 0;
                        if(cnt_conv_raddr == 39)  cnt_conv_raddr_n = 36;
                        if(cnt_conv_raddr == 75)  cnt_conv_raddr_n = 72;
                        if(cnt_conv_raddr == 111) cnt_conv_raddr_n = 108;
                    end

                    2'b01: begin
                        cnt_conv_raddr_n = cnt_conv_raddr + 1;
                        if(cnt_conv_raddr == 11)  cnt_conv_raddr_n = 0;
                        if(cnt_conv_raddr == 47)  cnt_conv_raddr_n = 36;
                        if(cnt_conv_raddr == 83)  cnt_conv_raddr_n = 72;
                        if(cnt_conv_raddr == 119) cnt_conv_raddr_n = 108;
                        if(cnt_conv_raddr == 155) cnt_conv_raddr_n = 144;
                        if(cnt_conv_raddr == 191) cnt_conv_raddr_n = 180;
                        if(cnt_conv_raddr == 227) cnt_conv_raddr_n = 216;
                        if(cnt_conv_raddr == 263) cnt_conv_raddr_n = 252;
                        if(cnt_conv_raddr == 299) cnt_conv_raddr_n = 288;
                        if(cnt_conv_raddr == 335) cnt_conv_raddr_n = 324;
                        if(cnt_conv_raddr == 371) cnt_conv_raddr_n = 360;
                        if(cnt_conv_raddr == 407) cnt_conv_raddr_n = 396;
                    end 
                    
                    2'b10: begin
                        cnt_conv_raddr_n = cnt_conv_raddr + 1;
                        if(cnt_conv_raddr == 27)  cnt_conv_raddr_n = 0;
                        if(cnt_conv_raddr == 63)  cnt_conv_raddr_n = 36;
                        if(cnt_conv_raddr == 99)  cnt_conv_raddr_n = 72;
                        if(cnt_conv_raddr == 135) cnt_conv_raddr_n = 108;
                        if(cnt_conv_raddr == 171) cnt_conv_raddr_n = 144;
                        if(cnt_conv_raddr == 207) cnt_conv_raddr_n = 180;
                        if(cnt_conv_raddr == 243) cnt_conv_raddr_n = 216;
                        if(cnt_conv_raddr == 279) cnt_conv_raddr_n = 252;
                        if(cnt_conv_raddr == 315) cnt_conv_raddr_n = 288;
                        if(cnt_conv_raddr == 351) cnt_conv_raddr_n = 324;
                        if(cnt_conv_raddr == 387) cnt_conv_raddr_n = 360;
                        if(cnt_conv_raddr == 423) cnt_conv_raddr_n = 396;
                        if(cnt_conv_raddr == 459) cnt_conv_raddr_n = 432;
                        if(cnt_conv_raddr == 495) cnt_conv_raddr_n = 468;
                        if(cnt_conv_raddr == 531) cnt_conv_raddr_n = 504;
                        if(cnt_conv_raddr == 567) cnt_conv_raddr_n = 540;
                        if(cnt_conv_raddr == 603) cnt_conv_raddr_n = 576;
                        if(cnt_conv_raddr == 639) cnt_conv_raddr_n = 612;
                        if(cnt_conv_raddr == 675) cnt_conv_raddr_n = 648;
                        if(cnt_conv_raddr == 711) cnt_conv_raddr_n = 684;
                        if(cnt_conv_raddr == 747) cnt_conv_raddr_n = 720;
                        if(cnt_conv_raddr == 783) cnt_conv_raddr_n = 756;
                        if(cnt_conv_raddr == 819) cnt_conv_raddr_n = 792;
                        if(cnt_conv_raddr == 855) cnt_conv_raddr_n = 828;
                        if(cnt_conv_raddr == 891) cnt_conv_raddr_n = 864;
                        if(cnt_conv_raddr == 927) cnt_conv_raddr_n = 900;
                        if(cnt_conv_raddr == 963) cnt_conv_raddr_n = 936;
                        if(cnt_conv_raddr == 999) cnt_conv_raddr_n = 972;
                    end
                    default: cnt_conv_raddr_n = 0;
                endcase
            end
		end
        
        ST_DECONV: begin
            if(mat_len == 2'b00 && cnt_size == 11 || mat_len == 2'b01 && cnt_size == 19 || mat_len == 2'b10 && cnt_size == 35) begin
                case (cnt_row)
                    1:  cnt_conv_raddr_n = 36;
                    2:  cnt_conv_raddr_n = 72;
                    3:  cnt_conv_raddr_n = 108;
                    4:  cnt_conv_raddr_n = 144;
                    5:  cnt_conv_raddr_n = 180;
                    6:  cnt_conv_raddr_n = 216;
                    7:  cnt_conv_raddr_n = 252;
                    8:  cnt_conv_raddr_n = 288;
                    9:  cnt_conv_raddr_n = 324;
                    10: cnt_conv_raddr_n = 360;
                    11: cnt_conv_raddr_n = 396;
                    12: cnt_conv_raddr_n = 432;
                    13: cnt_conv_raddr_n = 468;
                    14: cnt_conv_raddr_n = 504;
                    15: cnt_conv_raddr_n = 540;
                    16: cnt_conv_raddr_n = 576;
                    17: cnt_conv_raddr_n = 612;
                    18: cnt_conv_raddr_n = 648;
                    19: cnt_conv_raddr_n = 684;
                    20: cnt_conv_raddr_n = 720;
                    21: cnt_conv_raddr_n = 756;
                    22: cnt_conv_raddr_n = 792;
                    23: cnt_conv_raddr_n = 828;
                    23: cnt_conv_raddr_n = 828;
                    24: cnt_conv_raddr_n = 864;
                    25: cnt_conv_raddr_n = 900;
                    26: cnt_conv_raddr_n = 936;
                    27: cnt_conv_raddr_n = 972;
                    28: cnt_conv_raddr_n = 1008;
                    29: cnt_conv_raddr_n = 1044;
                    30: cnt_conv_raddr_n = 1080;
                    31: cnt_conv_raddr_n = 1116;
                    32: cnt_conv_raddr_n = 1152;
                    33: cnt_conv_raddr_n = 1188;
                    34: cnt_conv_raddr_n = 1224;
                    35: cnt_conv_raddr_n = 1260;
                    default: cnt_conv_raddr_n = 0;
                endcase
            end
            else begin
                cnt_conv_raddr_n = cnt_conv_raddr + 1;
            end
        end
        default: cnt_conv_raddr_n = 0;
    endcase
end

always @(*) begin
    case(state)
        ST_CONV: begin  // conv addr
            if(cnt == 2) begin
                case (cnt_addr) // MAT_A offset
                    1:  cnt_conv_waddr_n = 36;
                    2:  cnt_conv_waddr_n = 72;
                    3:  cnt_conv_waddr_n = 108;
                    4:  cnt_conv_waddr_n = 144;
                    5:  cnt_conv_waddr_n = 180;
                    6:  cnt_conv_waddr_n = 216;
                    7:  cnt_conv_waddr_n = 252;
                    8:  cnt_conv_waddr_n = 288;
                    9:  cnt_conv_waddr_n = 324;
                    10: cnt_conv_waddr_n = 360;
                    11: cnt_conv_waddr_n = 396;
                    12: cnt_conv_waddr_n = 432;
                    13: cnt_conv_waddr_n = 468;
                    14: cnt_conv_waddr_n = 504;
                    15: cnt_conv_waddr_n = 540;
                    16: cnt_conv_waddr_n = 576;
                    17: cnt_conv_waddr_n = 612;
                    18: cnt_conv_waddr_n = 648;
                    19: cnt_conv_waddr_n = 684;
                    20: cnt_conv_waddr_n = 720;
                    21: cnt_conv_waddr_n = 756;
                    22: cnt_conv_waddr_n = 792;
                    23: cnt_conv_waddr_n = 828;
                    24: cnt_conv_waddr_n = 864;
                    25: cnt_conv_waddr_n = 900;
                    26: cnt_conv_waddr_n = 936;
                    27: cnt_conv_waddr_n = 972;
                    default: cnt_conv_waddr_n = 0;
                endcase
            end
            else begin
                case(mat_len)
                    2'b00:  begin
                        cnt_conv_waddr_n = cnt_conv_waddr + 1;
                        if(cnt_conv_waddr == 3)   cnt_conv_waddr_n = 0;
                        if(cnt_conv_waddr == 39)  cnt_conv_waddr_n = 36;
                        if(cnt_conv_waddr == 75)  cnt_conv_waddr_n = 72;
                        if(cnt_conv_waddr == 111) cnt_conv_waddr_n = 108;
                    end

                    2'b01: begin
                        cnt_conv_waddr_n = cnt_conv_waddr + 1;
                        if(cnt_conv_waddr == 11)  cnt_conv_waddr_n = 0;
                        if(cnt_conv_waddr == 47)  cnt_conv_waddr_n = 36;
                        if(cnt_conv_waddr == 83)  cnt_conv_waddr_n = 72;
                        if(cnt_conv_waddr == 119) cnt_conv_waddr_n = 108;
                        if(cnt_conv_waddr == 155) cnt_conv_waddr_n = 144;
                        if(cnt_conv_waddr == 191) cnt_conv_waddr_n = 180;
                        if(cnt_conv_waddr == 227) cnt_conv_waddr_n = 216;
                        if(cnt_conv_waddr == 263) cnt_conv_waddr_n = 252;
                        if(cnt_conv_waddr == 299) cnt_conv_waddr_n = 288;
                        if(cnt_conv_waddr == 335) cnt_conv_waddr_n = 324;
                        if(cnt_conv_waddr == 371) cnt_conv_waddr_n = 360;
                        if(cnt_conv_waddr == 407) cnt_conv_waddr_n = 396;
                    end 
                    
                    2'b10: begin
                        cnt_conv_waddr_n = cnt_conv_waddr + 1;
                        if(cnt_conv_waddr == 27)  cnt_conv_waddr_n = 0;
                        if(cnt_conv_waddr == 63)  cnt_conv_waddr_n = 36;
                        if(cnt_conv_waddr == 99)  cnt_conv_waddr_n = 72;
                        if(cnt_conv_waddr == 135) cnt_conv_waddr_n = 108;
                        if(cnt_conv_waddr == 171) cnt_conv_waddr_n = 144;
                        if(cnt_conv_waddr == 207) cnt_conv_waddr_n = 180;
                        if(cnt_conv_waddr == 243) cnt_conv_waddr_n = 216;
                        if(cnt_conv_waddr == 279) cnt_conv_waddr_n = 252;
                        if(cnt_conv_waddr == 315) cnt_conv_waddr_n = 288;
                        if(cnt_conv_waddr == 351) cnt_conv_waddr_n = 324;
                        if(cnt_conv_waddr == 387) cnt_conv_waddr_n = 360;
                        if(cnt_conv_waddr == 423) cnt_conv_waddr_n = 396;
                        if(cnt_conv_waddr == 459) cnt_conv_waddr_n = 432;
                        if(cnt_conv_waddr == 495) cnt_conv_waddr_n = 468;
                        if(cnt_conv_waddr == 531) cnt_conv_waddr_n = 504;
                        if(cnt_conv_waddr == 567) cnt_conv_waddr_n = 540;
                        if(cnt_conv_waddr == 603) cnt_conv_waddr_n = 576;
                        if(cnt_conv_waddr == 639) cnt_conv_waddr_n = 612;
                        if(cnt_conv_waddr == 675) cnt_conv_waddr_n = 648;
                        if(cnt_conv_waddr == 711) cnt_conv_waddr_n = 684;
                        if(cnt_conv_waddr == 747) cnt_conv_waddr_n = 720;
                        if(cnt_conv_waddr == 783) cnt_conv_waddr_n = 756;
                        if(cnt_conv_waddr == 819) cnt_conv_waddr_n = 792;
                        if(cnt_conv_waddr == 855) cnt_conv_waddr_n = 828;
                        if(cnt_conv_waddr == 891) cnt_conv_waddr_n = 864;
                        if(cnt_conv_waddr == 927) cnt_conv_waddr_n = 900;
                        if(cnt_conv_waddr == 963) cnt_conv_waddr_n = 936;
                        if(cnt_conv_waddr == 999) cnt_conv_waddr_n = 972;
                    end
                    default: cnt_conv_waddr_n = 0;
                endcase
            end
		end

        ST_DECONV: begin
            if(mat_len == 2'b00 && cnt_size == 2 || mat_len == 2'b01 && cnt_size == 2 || mat_len == 2'b10 && cnt_size == 2) begin
                case (cnt_row)
                    1:  cnt_conv_waddr_n = 36;
                    2:  cnt_conv_waddr_n = 72;
                    3:  cnt_conv_waddr_n = 108;
                    4:  cnt_conv_waddr_n = 144;
                    5:  cnt_conv_waddr_n = 180;
                    6:  cnt_conv_waddr_n = 216;
                    7:  cnt_conv_waddr_n = 252;
                    8:  cnt_conv_waddr_n = 288;
                    9:  cnt_conv_waddr_n = 324;
                    10: cnt_conv_waddr_n = 360;
                    11: cnt_conv_waddr_n = 396;
                    12: cnt_conv_waddr_n = 432;
                    13: cnt_conv_waddr_n = 468;
                    14: cnt_conv_waddr_n = 504;
                    15: cnt_conv_waddr_n = 540;
                    16: cnt_conv_waddr_n = 576;
                    17: cnt_conv_waddr_n = 612;
                    18: cnt_conv_waddr_n = 648;
                    19: cnt_conv_waddr_n = 684;
                    20: cnt_conv_waddr_n = 720;
                    21: cnt_conv_waddr_n = 756;
                    22: cnt_conv_waddr_n = 792;
                    23: cnt_conv_waddr_n = 828;
                    23: cnt_conv_waddr_n = 828;
                    24: cnt_conv_waddr_n = 864;
                    25: cnt_conv_waddr_n = 900;
                    26: cnt_conv_waddr_n = 936;
                    27: cnt_conv_waddr_n = 972;
                    28: cnt_conv_waddr_n = 1008;
                    29: cnt_conv_waddr_n = 1044;
                    30: cnt_conv_waddr_n = 1080;
                    31: cnt_conv_waddr_n = 1116;
                    32: cnt_conv_waddr_n = 1152;
                    33: cnt_conv_waddr_n = 1188;
                    34: cnt_conv_waddr_n = 1224;
                    35: cnt_conv_waddr_n = 1260;
                    default: cnt_conv_waddr_n = 0;
                endcase
            end
            else begin
                cnt_conv_waddr_n = cnt_conv_waddr + 1;
            end
        end
        
        default: cnt_conv_waddr_n = 0;
    endcase
end

always @(*) begin
    if((state == ST_CONV || state == ST_DECONV) && cnt_size[1])
        conv_web_reg_n = 1'b0;
    else if(state == ST_CONV || state == ST_DECONV)
        conv_web_reg_n = conv_web_reg;
    else
        conv_web_reg_n = 1'b1;

end

always @(*) begin
    case(state)
		ST_IDLE: begin
			cnt_size_n = 0;
		end	
		ST_STORAGE:	begin
			case(mat_len)
                2'b00:
                    if(&cnt_size[2:0]) cnt_size_n = 0;    // if cnt_size = 7
                    else               cnt_size_n = cnt_size + 1;
                2'b01:
                    if(&cnt_size[3:0]) cnt_size_n = 0;    // if cnt_size = 15
                    else               cnt_size_n = cnt_size + 1;
                2'b10:
                    if(&cnt_size[4:0]) cnt_size_n = 0;    // if cnt_size = 32
                    else               cnt_size_n = cnt_size + 1;
                
                default: cnt_size_n = cnt_size;
			endcase
		end

        ST_KERNEL: begin
            if(cnt_size[2]) cnt_size_n = 0;
            else            cnt_size_n = cnt_size + 1'b1;
        end

        ST_CONV: begin
            case(mat_len)
                2'b00: begin
                    if(cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 3
                    else                           cnt_size_n = cnt_size + 1'b1;
                end
                2'b01: begin
                    if(cnt_size[3] && !cnt_size[2] && cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 11
                    else                                                          cnt_size_n = cnt_size + 1'b1;
                end
                2'b10: begin
                    if(cnt_size[4] && cnt_size[3] && !cnt_size[2] && cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 27
                    else                                                                         cnt_size_n = cnt_size + 1'b1;
                end
                default: cnt_size_n = cnt_size + 1'b1;
			endcase
        end

        ST_DECONV: begin
            case(mat_len)
                2'b00: begin
                    if(cnt_size[3] && !cnt_size[2] && cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 11
                    else  cnt_size_n = cnt_size + 1'b1;
                end
                2'b01: begin
                    if(cnt_size[4] && !cnt_size[3] && !cnt_size[2] && cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 19
                    else  cnt_size_n = cnt_size + 1'b1;
                end
                2'b10: begin
                    if(cnt_size[5] && !cnt_size[4] && !cnt_size[3] && !cnt_size[2] && cnt_size[1] && cnt_size[0]) cnt_size_n = 0;    // if cnt size = 35
                    else  cnt_size_n = cnt_size + 1'b1;
                end
                default: cnt_size_n = cnt_size + 1'b1;
			endcase
        end
        default: cnt_size_n = 0;
    endcase
end

always @(*) begin
    case(state)
        ST_DECONV: begin
            if(done_layer_0 || done_layer_1 || done_layer_2 || done_layer_3 || done_layer_4 || 
                done_layer_0_b || done_layer_1_b || done_layer_2_b || done_layer_3_b)
                cnt_row_n = cnt_row + 1;
            else
                cnt_row_n = cnt_row;
        end
        default: cnt_row_n = 0;
    endcase    
end

SRAM_512X256 U_MAT (.A(MAT_A), .DI(MAT_DI), .CK(clk), .WEB(MAT_WEB), .DO(MAT_DO));
SRAM_80X40_KERNEL U_KER  (.A(KER_A), .DI(KER_DI), .CK(clk), .WEB(KER_WEB), .DO(KER_DO));
SRAM_1296X20_CONV U_CONV (.CK(clk), .WEB(CONV_WEB), .RA(CONV_RA), .WA(CONV_WA), .DI(CONV_DI), .DO(CONV_DO));
SRAM_1296X20_CONV U_OUT (.CK(clk), .WEB(OUT_WEB), .RA(OUT_RA), .WA(OUT_WA), .DI(OUT_DI), .DO(OUT_DO));
endmodule


module SRAM_512X256 (
    A,
    DI,
    CK,
    WEB,
	DO
    );

// SRAM
input CK, WEB;
input    [8:0] A; 
input  [255:0] DI;

output [255:0] DO;

SRAM_512X128 U_0 (
    .CK(CK),   .CS(1'b1), .OE(1'b1), .WEB(WEB),
    .A0(A[0]),       .A1(A[1]),       .A2(A[2]),       .A3(A[3]),       .A4(A[4]),       .A5(A[5]),       .A6(A[6]),       .A7(A[7]),   .A8(A[8]),
    .DI0(DI[0]),     .DI1(DI[1]),     .DI2(DI[2]),     .DI3(DI[3]),     .DI4(DI[4]),     .DI5(DI[5]),     .DI6(DI[6]),     .DI7(DI[7]),
    .DI8(DI[8]),     .DI9(DI[9]),     .DI10(DI[10]),   .DI11(DI[11]),   .DI12(DI[12]),   .DI13(DI[13]),   .DI14(DI[14]),   .DI15(DI[15]),
    .DI16(DI[16]),   .DI17(DI[17]),   .DI18(DI[18]),   .DI19(DI[19]),   .DI20(DI[20]),   .DI21(DI[21]),   .DI22(DI[22]),   .DI23(DI[23]),
    .DI24(DI[24]),   .DI25(DI[25]),   .DI26(DI[26]),   .DI27(DI[27]),   .DI28(DI[28]),   .DI29(DI[29]),   .DI30(DI[30]),   .DI31(DI[31]),
    .DI32(DI[32]),   .DI33(DI[33]),   .DI34(DI[34]),   .DI35(DI[35]),   .DI36(DI[36]),   .DI37(DI[37]),   .DI38(DI[38]),   .DI39(DI[39]),
    .DI40(DI[40]),   .DI41(DI[41]),   .DI42(DI[42]),   .DI43(DI[43]),   .DI44(DI[44]),   .DI45(DI[45]),   .DI46(DI[46]),   .DI47(DI[47]),
    .DI48(DI[48]),   .DI49(DI[49]),   .DI50(DI[50]),   .DI51(DI[51]),   .DI52(DI[52]),   .DI53(DI[53]),   .DI54(DI[54]),   .DI55(DI[55]),
    .DI56(DI[56]),   .DI57(DI[57]),   .DI58(DI[58]),   .DI59(DI[59]),   .DI60(DI[60]),   .DI61(DI[61]),   .DI62(DI[62]),   .DI63(DI[63]),
    .DI64(DI[64]),   .DI65(DI[65]),   .DI66(DI[66]),   .DI67(DI[67]),   .DI68(DI[68]),   .DI69(DI[69]),   .DI70(DI[70]),   .DI71(DI[71]),
    .DI72(DI[72]),   .DI73(DI[73]),   .DI74(DI[74]),   .DI75(DI[75]),   .DI76(DI[76]),   .DI77(DI[77]),   .DI78(DI[78]),   .DI79(DI[79]),
    .DI80(DI[80]),   .DI81(DI[81]),   .DI82(DI[82]),   .DI83(DI[83]),   .DI84(DI[84]),   .DI85(DI[85]),   .DI86(DI[86]),   .DI87(DI[87]),
    .DI88(DI[88]),   .DI89(DI[89]),   .DI90(DI[90]),   .DI91(DI[91]),   .DI92(DI[92]),   .DI93(DI[93]),   .DI94(DI[94]),   .DI95(DI[95]),
    .DI96(DI[96]),   .DI97(DI[97]),   .DI98(DI[98]),   .DI99(DI[99]),   .DI100(DI[100]), .DI101(DI[101]), .DI102(DI[102]), .DI103(DI[103]),
    .DI104(DI[104]), .DI105(DI[105]), .DI106(DI[106]), .DI107(DI[107]), .DI108(DI[108]), .DI109(DI[109]), .DI110(DI[110]), .DI111(DI[111]),
    .DI112(DI[112]), .DI113(DI[113]), .DI114(DI[114]), .DI115(DI[115]), .DI116(DI[116]), .DI117(DI[117]), .DI118(DI[118]), .DI119(DI[119]),
    .DI120(DI[120]), .DI121(DI[121]), .DI122(DI[122]), .DI123(DI[123]), .DI124(DI[124]), .DI125(DI[125]), .DI126(DI[126]), .DI127(DI[127]),
    .DO0(DO[0]),     .DO1(DO[1]),     .DO2(DO[2]),     .DO3(DO[3]),     .DO4(DO[4]),     .DO5(DO[5]),     .DO6(DO[6]),     .DO7(DO[7]),
    .DO8(DO[8]),     .DO9(DO[9]),     .DO10(DO[10]),   .DO11(DO[11]),   .DO12(DO[12]),   .DO13(DO[13]),   .DO14(DO[14]),   .DO15(DO[15]),
    .DO16(DO[16]),   .DO17(DO[17]),   .DO18(DO[18]),   .DO19(DO[19]),   .DO20(DO[20]),   .DO21(DO[21]),   .DO22(DO[22]),   .DO23(DO[23]),
    .DO24(DO[24]),   .DO25(DO[25]),   .DO26(DO[26]),   .DO27(DO[27]),   .DO28(DO[28]),   .DO29(DO[29]),   .DO30(DO[30]),   .DO31(DO[31]),
    .DO32(DO[32]),   .DO33(DO[33]),   .DO34(DO[34]),   .DO35(DO[35]),   .DO36(DO[36]),   .DO37(DO[37]),   .DO38(DO[38]),   .DO39(DO[39]),
    .DO40(DO[40]),   .DO41(DO[41]),   .DO42(DO[42]),   .DO43(DO[43]),   .DO44(DO[44]),   .DO45(DO[45]),   .DO46(DO[46]),   .DO47(DO[47]),
    .DO48(DO[48]),   .DO49(DO[49]),   .DO50(DO[50]),   .DO51(DO[51]),   .DO52(DO[52]),   .DO53(DO[53]),   .DO54(DO[54]),   .DO55(DO[55]),
    .DO56(DO[56]),   .DO57(DO[57]),   .DO58(DO[58]),   .DO59(DO[59]),   .DO60(DO[60]),   .DO61(DO[61]),   .DO62(DO[62]),   .DO63(DO[63]),
    .DO64(DO[64]),   .DO65(DO[65]),   .DO66(DO[66]),   .DO67(DO[67]),   .DO68(DO[68]),   .DO69(DO[69]),   .DO70(DO[70]),   .DO71(DO[71]),
    .DO72(DO[72]),   .DO73(DO[73]),   .DO74(DO[74]),   .DO75(DO[75]),   .DO76(DO[76]),   .DO77(DO[77]),   .DO78(DO[78]),   .DO79(DO[79]),
    .DO80(DO[80]),   .DO81(DO[81]),   .DO82(DO[82]),   .DO83(DO[83]),   .DO84(DO[84]),   .DO85(DO[85]),   .DO86(DO[86]),   .DO87(DO[87]),
    .DO88(DO[88]),   .DO89(DO[89]),   .DO90(DO[90]),   .DO91(DO[91]),   .DO92(DO[92]),   .DO93(DO[93]),   .DO94(DO[94]),   .DO95(DO[95]),
    .DO96(DO[96]),   .DO97(DO[97]),   .DO98(DO[98]),   .DO99(DO[99]),   .DO100(DO[100]), .DO101(DO[101]), .DO102(DO[102]), .DO103(DO[103]),
    .DO104(DO[104]), .DO105(DO[105]), .DO106(DO[106]), .DO107(DO[107]), .DO108(DO[108]), .DO109(DO[109]), .DO110(DO[110]), .DO111(DO[111]),
    .DO112(DO[112]), .DO113(DO[113]), .DO114(DO[114]), .DO115(DO[115]), .DO116(DO[116]), .DO117(DO[117]), .DO118(DO[118]), .DO119(DO[119]),
    .DO120(DO[120]), .DO121(DO[121]), .DO122(DO[122]), .DO123(DO[123]), .DO124(DO[124]), .DO125(DO[125]), .DO126(DO[126]), .DO127(DO[127]));
    
SRAM_512X128 U_1 (
    .CK(CK),   .CS(1'b1), .OE(1'b1), .WEB(WEB),
    .A0(A[0]), .A1(A[1]), .A2(A[2]), .A3(A[3]), .A4(A[4]), .A5(A[5]), .A6(A[6]), .A7(A[7]), .A8(A[8]),
    .DI0(DI[128]), .DI1(DI[129]), .DI2(DI[130]), .DI3(DI[131]), .DI4(DI[132]), .DI5(DI[133]), .DI6(DI[134]), .DI7(DI[135]),
    .DI8(DI[136]), .DI9(DI[137]), .DI10(DI[138]), .DI11(DI[139]), .DI12(DI[140]), .DI13(DI[141]), .DI14(DI[142]), .DI15(DI[143]),
    .DI16(DI[144]), .DI17(DI[145]), .DI18(DI[146]), .DI19(DI[147]), .DI20(DI[148]), .DI21(DI[149]), .DI22(DI[150]), .DI23(DI[151]),
    .DI24(DI[152]), .DI25(DI[153]), .DI26(DI[154]), .DI27(DI[155]), .DI28(DI[156]), .DI29(DI[157]), .DI30(DI[158]), .DI31(DI[159]),
    .DI32(DI[160]), .DI33(DI[161]), .DI34(DI[162]), .DI35(DI[163]), .DI36(DI[164]), .DI37(DI[165]), .DI38(DI[166]), .DI39(DI[167]),
    .DI40(DI[168]), .DI41(DI[169]), .DI42(DI[170]), .DI43(DI[171]), .DI44(DI[172]), .DI45(DI[173]), .DI46(DI[174]), .DI47(DI[175]),
    .DI48(DI[176]), .DI49(DI[177]), .DI50(DI[178]), .DI51(DI[179]), .DI52(DI[180]), .DI53(DI[181]), .DI54(DI[182]), .DI55(DI[183]),
    .DI56(DI[184]), .DI57(DI[185]), .DI58(DI[186]), .DI59(DI[187]), .DI60(DI[188]), .DI61(DI[189]), .DI62(DI[190]), .DI63(DI[191]),
    .DI64(DI[192]), .DI65(DI[193]), .DI66(DI[194]), .DI67(DI[195]), .DI68(DI[196]), .DI69(DI[197]), .DI70(DI[198]), .DI71(DI[199]),
    .DI72(DI[200]), .DI73(DI[201]), .DI74(DI[202]), .DI75(DI[203]), .DI76(DI[204]), .DI77(DI[205]), .DI78(DI[206]), .DI79(DI[207]),
    .DI80(DI[208]), .DI81(DI[209]), .DI82(DI[210]), .DI83(DI[211]), .DI84(DI[212]), .DI85(DI[213]), .DI86(DI[214]), .DI87(DI[215]),
    .DI88(DI[216]), .DI89(DI[217]), .DI90(DI[218]), .DI91(DI[219]), .DI92(DI[220]), .DI93(DI[221]), .DI94(DI[222]), .DI95(DI[223]),
    .DI96(DI[224]), .DI97(DI[225]), .DI98(DI[226]), .DI99(DI[227]), .DI100(DI[228]), .DI101(DI[229]), .DI102(DI[230]), .DI103(DI[231]),
    .DI104(DI[232]), .DI105(DI[233]), .DI106(DI[234]), .DI107(DI[235]), .DI108(DI[236]), .DI109(DI[237]), .DI110(DI[238]), .DI111(DI[239]),
    .DI112(DI[240]), .DI113(DI[241]), .DI114(DI[242]), .DI115(DI[243]), .DI116(DI[244]), .DI117(DI[245]), .DI118(DI[246]), .DI119(DI[247]),
    .DI120(DI[248]), .DI121(DI[249]), .DI122(DI[250]), .DI123(DI[251]), .DI124(DI[252]), .DI125(DI[253]), .DI126(DI[254]), .DI127(DI[255]),
    .DO0(DO[128]), .DO1(DO[129]), .DO2(DO[130]), .DO3(DO[131]), .DO4(DO[132]), .DO5(DO[133]), .DO6(DO[134]), .DO7(DO[135]),
    .DO8(DO[136]), .DO9(DO[137]), .DO10(DO[138]), .DO11(DO[139]), .DO12(DO[140]), .DO13(DO[141]), .DO14(DO[142]), .DO15(DO[143]),
    .DO16(DO[144]), .DO17(DO[145]), .DO18(DO[146]), .DO19(DO[147]), .DO20(DO[148]), .DO21(DO[149]), .DO22(DO[150]), .DO23(DO[151]),
    .DO24(DO[152]), .DO25(DO[153]), .DO26(DO[154]), .DO27(DO[155]), .DO28(DO[156]), .DO29(DO[157]), .DO30(DO[158]), .DO31(DO[159]),
    .DO32(DO[160]), .DO33(DO[161]), .DO34(DO[162]), .DO35(DO[163]), .DO36(DO[164]), .DO37(DO[165]), .DO38(DO[166]), .DO39(DO[167]),
    .DO40(DO[168]), .DO41(DO[169]), .DO42(DO[170]), .DO43(DO[171]), .DO44(DO[172]), .DO45(DO[173]), .DO46(DO[174]), .DO47(DO[175]),
    .DO48(DO[176]), .DO49(DO[177]), .DO50(DO[178]), .DO51(DO[179]), .DO52(DO[180]), .DO53(DO[181]), .DO54(DO[182]), .DO55(DO[183]),
    .DO56(DO[184]), .DO57(DO[185]), .DO58(DO[186]), .DO59(DO[187]), .DO60(DO[188]), .DO61(DO[189]), .DO62(DO[190]), .DO63(DO[191]),
    .DO64(DO[192]), .DO65(DO[193]), .DO66(DO[194]), .DO67(DO[195]), .DO68(DO[196]), .DO69(DO[197]), .DO70(DO[198]), .DO71(DO[199]),
    .DO72(DO[200]), .DO73(DO[201]), .DO74(DO[202]), .DO75(DO[203]), .DO76(DO[204]), .DO77(DO[205]), .DO78(DO[206]), .DO79(DO[207]),
    .DO80(DO[208]), .DO81(DO[209]), .DO82(DO[210]), .DO83(DO[211]), .DO84(DO[212]), .DO85(DO[213]), .DO86(DO[214]), .DO87(DO[215]),
    .DO88(DO[216]), .DO89(DO[217]), .DO90(DO[218]), .DO91(DO[219]), .DO92(DO[220]), .DO93(DO[221]), .DO94(DO[222]), .DO95(DO[223]),
    .DO96(DO[224]), .DO97(DO[225]), .DO98(DO[226]), .DO99(DO[227]), .DO100(DO[228]), .DO101(DO[229]), .DO102(DO[230]), .DO103(DO[231]),
    .DO104(DO[232]), .DO105(DO[233]), .DO106(DO[234]), .DO107(DO[235]), .DO108(DO[236]), .DO109(DO[237]), .DO110(DO[238]), .DO111(DO[239]),
    .DO112(DO[240]), .DO113(DO[241]), .DO114(DO[242]), .DO115(DO[243]), .DO116(DO[244]), .DO117(DO[245]), .DO118(DO[246]), .DO119(DO[247]),
    .DO120(DO[248]), .DO121(DO[249]), .DO122(DO[250]), .DO123(DO[251]), .DO124(DO[252]), .DO125(DO[253]), .DO126(DO[254]), .DO127(DO[255]));

endmodule

module SRAM_80X40_KERNEL (
    A,
    DI,
    CK,
    WEB,
	DO
    );

// SRAM
input CK, WEB;
input  [7-1:0] A; 
input  [40-1:0] DI;

output [40-1:0] DO;

SRAM_80X40 U_0 (
    .CK(CK),   .CS(1'b1), .OE(1'b1), .WEB(WEB),
    .A0(A[0]),       .A1(A[1]),       .A2(A[2]),       .A3(A[3]),       .A4(A[4]),       .A5(A[5]),       .A6(A[6]),
    .DI0(DI[0]),     .DI1(DI[1]),     .DI2(DI[2]),     .DI3(DI[3]),     .DI4(DI[4]),     .DI5(DI[5]),     .DI6(DI[6]),     .DI7(DI[7]),
    .DI8(DI[8]),     .DI9(DI[9]),     .DI10(DI[10]),   .DI11(DI[11]),   .DI12(DI[12]),   .DI13(DI[13]),   .DI14(DI[14]),   .DI15(DI[15]),
    .DI16(DI[16]),   .DI17(DI[17]),   .DI18(DI[18]),   .DI19(DI[19]),   .DI20(DI[20]),   .DI21(DI[21]),   .DI22(DI[22]),   .DI23(DI[23]),
    .DI24(DI[24]),   .DI25(DI[25]),   .DI26(DI[26]),   .DI27(DI[27]),   .DI28(DI[28]),   .DI29(DI[29]),   .DI30(DI[30]),   .DI31(DI[31]),
    .DI32(DI[32]),   .DI33(DI[33]),   .DI34(DI[34]),   .DI35(DI[35]),   .DI36(DI[36]),   .DI37(DI[37]),   .DI38(DI[38]),   .DI39(DI[39]),
    
    .DO0(DO[0]),     .DO1(DO[1]),     .DO2(DO[2]),     .DO3(DO[3]),     .DO4(DO[4]),     .DO5(DO[5]),     .DO6(DO[6]),     .DO7(DO[7]),
    .DO8(DO[8]),     .DO9(DO[9]),     .DO10(DO[10]),   .DO11(DO[11]),   .DO12(DO[12]),   .DO13(DO[13]),   .DO14(DO[14]),   .DO15(DO[15]),
    .DO16(DO[16]),   .DO17(DO[17]),   .DO18(DO[18]),   .DO19(DO[19]),   .DO20(DO[20]),   .DO21(DO[21]),   .DO22(DO[22]),   .DO23(DO[23]),
    .DO24(DO[24]),   .DO25(DO[25]),   .DO26(DO[26]),   .DO27(DO[27]),   .DO28(DO[28]),   .DO29(DO[29]),   .DO30(DO[30]),   .DO31(DO[31]),
    .DO32(DO[32]),   .DO33(DO[33]),   .DO34(DO[34]),   .DO35(DO[35]),   .DO36(DO[36]),   .DO37(DO[37]),   .DO38(DO[38]),   .DO39(DO[39]));

endmodule

module SRAM_1296X20_CONV (
    CK,
    WEB,
    RA,
    WA,
    DI,
	DO
    );

// SRAM
input CK, WEB;
input  [11-1:0] RA, WA;
input  [20-1:0] DI;
output [20-1:0] DO;

// A    = Read addr
// WEBA = 1'b1
// DOA  = read out data

// B    = Write addr
// WEBN = WEB
// DIB  = write in data
SRAM_1296X20 U_0  (
    .CKA(CK), .CKB(CK),  .CSA(1'b1), .CSB(1'b1), .OEA(1'b1), .OEB(1'b1), .WEAN(1'b1), .WEBN(WEB),
    .A0(RA[0]), .A1(RA[1]), .A2(RA[2]), .A3(RA[3]), .A4(RA[4]), .A5(RA[5]), .A6(RA[6]), .A7(RA[7]), .A8(RA[8]), .A9(RA[9]), .A10(RA[10]),
    .B0(WA[0]), .B1(WA[1]), .B2(WA[2]), .B3(WA[3]), .B4(WA[4]), .B5(WA[5]), .B6(WA[6]), .B7(WA[7]), .B8(WA[8]), .B9(WA[9]), .B10(WA[10]),
    
    .DIB0(DI[0]),  .DIB1(DI[1]),  .DIB2(DI[2]),  .DIB3(DI[3]),
    .DIB4(DI[4]),  .DIB5(DI[5]),  .DIB6(DI[6]),  .DIB7(DI[7]),
    .DIB8(DI[8]),  .DIB9(DI[9]),  .DIB10(DI[10]), .DIB11(DI[11]),
    .DIB12(DI[12]), .DIB13(DI[13]), .DIB14(DI[14]), .DIB15(DI[15]),
    .DIB16(DI[16]), .DIB17(DI[17]), .DIB18(DI[18]), .DIB19(DI[19]),

    .DOA0(DO[0]),  .DOA1(DO[1]),  .DOA2(DO[2]),  .DOA3(DO[3]),
    .DOA4(DO[4]),  .DOA5(DO[5]),  .DOA6(DO[6]),  .DOA7(DO[7]),
    .DOA8(DO[8]),  .DOA9(DO[9]),  .DOA10(DO[10]), .DOA11(DO[11]),
    .DOA12(DO[12]), .DOA13(DO[13]), .DOA14(DO[14]), .DOA15(DO[15]),
    .DOA16(DO[16]), .DOA17(DO[17]), .DOA18(DO[18]), .DOA19(DO[19])
);
endmodule