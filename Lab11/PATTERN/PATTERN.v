//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   2024 ICLAB Spring Course
//   Lab11      : SNN
//   Author     : ZONG-RUI CAO
//                
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   DESCRIPTION: 2024 Spring IC Lab / Exercise Lab11 / SNN
//   Release version : v1.0 (Release Date: May-2024)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################
`define CYCLE_TIME   15

module PATTERN(
	// Output signals
	clk,
	rst_n,
	in_valid,
	img,
	ker,
	weight,

	// Input signals
	out_valid,
	out_data
);

output reg clk;
output reg rst_n;
output reg in_valid;
output reg [7:0] img;
output reg [7:0] ker;
output reg [7:0] weight;

input out_valid;
input  [9:0] out_data;

integer i,j,k,l,m,n,i_pat,total_latency,debug;

parameter PAT_NUM=10000;

always #(`CYCLE_TIME/2.0) clk = ~clk;
reg [7:0] img0[0:5][0:5],img1[0:5][0:5];
reg [7:0] ker_reg[0:2][0:2];
reg [7:0] weight_reg[0:1][0:1];
reg [9:0] ans;

//================================================================
// Clock
//================================================================

//================================================================
// parameters & integer
//================================================================

reg [10:0] cnt;
//================================================================
// Wire & Reg Declaration
//================================================================

reg [20:0] feature_map0[0:3][0:3],feature_map1[0:3][0:3];
reg [20:0] quan_prev_0[0:3][0:3],quan_prev_1[0:3][0:3];
reg [20:0] max_pool_0[0:1][0:1],max_pool_1[0:1][0:1];

reg [20:0] full_con_0[0:3],full_con_1[0:3];
reg [20:0] quan_second_0[0:3],quan_second_1[0:3];

reg [20:0] L1_distance;


task display_data; begin
	debug = $fopen("../00_TESTBED/debug.txt", "w");
	$fwrite(debug, "[PAT NO. %4d]\n\n", i_pat);
	$fwrite(debug, "img0:\n");
	for(i=0;i<6;i=i+1)begin
		for (j=0;j<6;j=j+1) begin
			$fwrite(debug, "%d ",img0[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "img1:\n");
	for(i=0;i<6;i=i+1)begin
		for (j=0;j<6;j=j+1) begin
			$fwrite(debug, "%d ",img1[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "ker0:\n");
	for(i=0;i<3;i=i+1)begin
		for (j=0;j<3;j=j+1) begin
			$fwrite(debug, "%d ",ker_reg[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "wei:\n");
	for(i=0;i<2;i=i+1)begin
		for (j=0;j<2;j=j+1) begin
			$fwrite(debug, "%d ",weight_reg[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "feature_map0:\n");
	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			$fwrite(debug, "%d ",feature_map0[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "feature_map1:\n");
	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			$fwrite(debug, "%d ",feature_map1[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "First Quantization_0:\n");
	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			$fwrite(debug, "%d ",quan_prev_0[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "First Quantization_1:\n");
	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			$fwrite(debug, "%d ",quan_prev_1[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "max_pool_0:\n");
	for(i=0;i<2;i=i+1)begin
		for(j=0;j<2;j=j+1)begin
			$fwrite(debug, "%d ",max_pool_0[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "max_pool_1:\n");
	for(i=0;i<2;i=i+1)begin
		for(j=0;j<2;j=j+1)begin
			$fwrite(debug, "%d ",max_pool_1[i][j]);
		end
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "ful_con_0:\n");
	for(i=0;i<4;i=i+1)begin
		$fwrite(debug, "%d ",full_con_0[i]);
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "ful_con_1:\n");
	for(i=0;i<4;i=i+1)begin
		$fwrite(debug, "%d ",full_con_1[i]);
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "Second Quantization_0:\n");
	for(i=0;i<4;i=i+1)begin
		$fwrite(debug, "%d ",quan_second_0[i]);
		$fwrite(debug, "\n");
	end	
	$fwrite(debug, "\n");
	$fwrite(debug, "Second Quantization_1:\n");
	for(i=0;i<4;i=i+1)begin
		$fwrite(debug, "%d ",quan_second_1[i]);
		$fwrite(debug, "\n");
	end
	$fwrite(debug, "\n");
	$fwrite(debug, "L1_distance:\n");
	$fwrite(debug, "%d\n",L1_distance);
	$fwrite(debug, "\n");
	$fwrite(debug, "ans:\n");
	$fwrite(debug, "%d\n",ans);

end
endtask

task cal_task;begin
	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			feature_map0[i][j]=img0[i][j]*ker_reg[0][0]+img0[i][j+1]*ker_reg[0][1]+img0[i][j+2]*ker_reg[0][2]
			+img0[i+1][j]*ker_reg[1][0]+img0[i+1][j+1]*ker_reg[1][1]+img0[i+1][j+2]*ker_reg[1][2]
			+img0[i+2][j]*ker_reg[2][0]+img0[i+2][j+1]*ker_reg[2][1]+img0[i+2][j+2]*ker_reg[2][2];

			feature_map1[i][j]=img1[i][j]*ker_reg[0][0]+img1[i][j+1]*ker_reg[0][1]+img1[i][j+2]*ker_reg[0][2]
			+img1[i+1][j]*ker_reg[1][0]+img1[i+1][j+1]*ker_reg[1][1]+img1[i+1][j+2]*ker_reg[1][2]
			+img1[i+2][j]*ker_reg[2][0]+img1[i+2][j+1]*ker_reg[2][1]+img1[i+2][j+2]*ker_reg[2][2];
		end
	end

	for(i=0;i<4;i=i+1)begin
		for(j=0;j<4;j=j+1)begin
			quan_prev_0[i][j]=feature_map0[i][j]/2295;
			quan_prev_1[i][j]=feature_map1[i][j]/2295;
		end
	end

	for(i=0;i<2;i=i+1)begin
		for(j=0;j<2;j=j+1)begin
			l=quan_prev_0[i*2][j*2];
			for(k=0;k<4;k=k+1)begin
				l=(l>quan_prev_0[i*2+k/2][j*2+k%2])?l:quan_prev_0[i*2+k/2][j*2+k%2];
			end			
			max_pool_0[i][j]=l;

			l=quan_prev_1[i*2][j*2];
			for(k=0;k<4;k=k+1)begin
				l=(l>quan_prev_1[i*2+k/2][j*2+k%2])?l:quan_prev_1[i*2+k/2][j*2+k%2];
			end	
			max_pool_1[i][j]=l;
		end
	end

	
	full_con_0[0]=max_pool_0[0][0]*weight_reg[0][0]+max_pool_0[0][1]*weight_reg[1][0];
	full_con_0[1]=max_pool_0[0][0]*weight_reg[0][1]+max_pool_0[0][1]*weight_reg[1][1];
	full_con_0[2]=max_pool_0[1][0]*weight_reg[0][0]+max_pool_0[1][1]*weight_reg[1][0];
	full_con_0[3]=max_pool_0[1][0]*weight_reg[0][1]+max_pool_0[1][1]*weight_reg[1][1];

	full_con_1[0]=max_pool_1[0][0]*weight_reg[0][0]+max_pool_1[0][1]*weight_reg[1][0];
	full_con_1[1]=max_pool_1[0][0]*weight_reg[0][1]+max_pool_1[0][1]*weight_reg[1][1];
	full_con_1[2]=max_pool_1[1][0]*weight_reg[0][0]+max_pool_1[1][1]*weight_reg[1][0];
	full_con_1[3]=max_pool_1[1][0]*weight_reg[0][1]+max_pool_1[1][1]*weight_reg[1][1];

	for(i=0;i<4;i=i+1)begin
		quan_second_0[i]=full_con_0[i]/510;
		quan_second_1[i]=full_con_1[i]/510;
	end

	L1_distance=0;

	for(i=0;i<4;i=i+1)begin
		k=(quan_second_0[i]>quan_second_1[i])?(quan_second_0[i]-quan_second_1[i]):(quan_second_1[i]-quan_second_0[i]);
		L1_distance=L1_distance+k;
	end

	ans=(L1_distance>=16)?L1_distance:0;

end
endtask

always @(posedge clk) begin
    if(out_valid===0)begin
        if(out_data!==0)begin
            $display("out should be 0 when out_valid is low");
            $finish;
        end
    end
end

always @(*) begin
    if(out_valid===1&&in_valid===1)begin
        $display("in_valid out_valid overlap");
            $finish;
    end
end

initial begin
    // $finish;

    reset_signal_task;

    i_pat = 0;

	// $display("con");
	
    for (i_pat = 1; i_pat <= PAT_NUM; i_pat = i_pat + 1) begin
        // $finish;
        input_task;
        // $finish;
        // wait_out_valid_task;
		// $display("con");
		// $finish;

        cal_task;
        // $finish;
        check_ans_task;

        @(negedge clk);
		@(negedge clk);
        $display("PASS PATTERN NO.%4d", i_pat);
    end
    $display("congratulation");
    $finish;


end

task check_ans_task;begin

	total_latency=0;
    while(out_valid==0)begin
		if(out_data!==0)begin
			$display("out should be zero");
			$finish;
		end
		@(negedge clk);
		if(total_latency>=900)begin
			display_data;
			$display("cycle exceed");
			$finish;
		end
		total_latency=total_latency+1;
	end

	if(out_valid===1)begin
		if(out_data!==ans)begin
			display_data;
			$display("ans_wrong");
			$display("golden_ans:%d",ans);
			$display("your_ans:%d",out_data);
			$finish;
		end
	end

	@(negedge clk);

	if(out_valid===1||out_data!==0)begin
      $display("out_valid exceed ");
      $finish;
    end

end
endtask



task input_task; begin
    
    img='dx;
    ker = 'dx;
	weight='dx;
    in_valid=0;
    for(i=0;i<72;i=i+1)begin
        in_valid=1;
        // 檢查文件是否成功打開
        // if (image_input != 0) begin
            // 使用 $fscanf 從文件中讀取數字
		if(i<9)begin
			ker=$urandom_range(0, 255);
			ker_reg[i/3][i%3]=ker;
		end
		else begin
			ker = 'dx;
		end

		if(i<4)begin
			weight=$urandom_range(0, 255);
			weight_reg[i/2][i%2]=weight;
		end
		else begin
			weight='dx;
		end

		if(i<36)begin
			img=$urandom_range(0, 255);
			img0[i/6][i%6]=img;
		end
		else begin
			img=$urandom_range(0, 255);
			img1[(i-36)/6][(i-36)%6]=img;
		end


        @(negedge clk);


    end
    in_valid=0;

    img='dx;
    ker = 'dx;
	weight='dx;
   

   
    

end endtask 



task reset_signal_task; begin


    force clk = 0;
    rst_n = 1;

    in_valid = 'd0;
   img='dx;
    ker = 'dx;
	weight='dx;

    // tot_lat = 0;

    #(`CYCLE_TIME/2.0) rst_n = 0;
    #(`CYCLE_TIME/2.0) rst_n = 1;
    if (out_valid !== 0 || out_data !== 0) begin
        $display("                                           `:::::`                                                       ");
        $display("                                          .+-----++                                                      ");
        $display("                .--.`                    o:------/o                                                      ");
        $display("              /+:--:o/                   //-------y.          -//:::-        `.`                         ");
        $display("            `/:------y:                  `o:--::::s/..``    `/:-----s-    .:/:::+:                       ");
        $display("            +:-------:y                `.-:+///::-::::://:-.o-------:o  `/:------s-                      ");
        $display("            y---------y-        ..--:::::------------------+/-------/+ `+:-------/s                      ");
        $display("           `s---------/s       +:/++/----------------------/+-------s.`o:--------/s                      ");
        $display("           .s----------y-      o-:----:---------------------/------o: +:---------o:                      ");
        $display("           `y----------:y      /:----:/-------/o+----------------:+- //----------y`                      ");
        $display("            y-----------o/ `.--+--/:-/+--------:+o--------------:o: :+----------/o                       ");
        $display("            s:----------:y/-::::::my-/:----------/---------------+:-o-----------y.                       ");
        $display("            -o----------s/-:hmmdy/o+/:---------------------------++o-----------/o                        ");
        $display("             s:--------/o--hMMMMMh---------:ho-------------------yo-----------:s`                        ");
        $display("             :o--------s/--hMMMMNs---------:hs------------------+s------------s-                         ");
        $display("              y:-------o+--oyhyo/-----------------------------:o+------------o-                          ");
        $display("              -o-------:y--/s--------------------------------/o:------------o/                           ");
        $display("               +/-------o+--++-----------:+/---------------:o/-------------+/                            ");
        $display("               `o:-------s:--/+:-------/o+-:------------::+d:-------------o/                             ");
        $display("                `o-------:s:---ohsoosyhh+----------:/+ooyhhh-------------o:                              ");
        $display("                 .o-------/d/--:h++ohy/---------:osyyyyhhyyd-----------:o-                               ");
        $display("                 .dy::/+syhhh+-::/::---------/osyyysyhhysssd+---------/o`                                ");
        $display("                  /shhyyyymhyys://-------:/oyyysyhyydysssssyho-------od:                                 ");
        $display("                    `:hhysymmhyhs/:://+osyyssssydyydyssssssssyyo+//+ymo`                                 ");
        $display("                      `+hyydyhdyyyyyyyyyyssssshhsshyssssssssssssyyyo:`                                   ");
        $display("                        -shdssyyyyyhhhhhyssssyyssshssssssssssssyy+.    Output signal should be 0         ");
        $display("                         `hysssyyyysssssssssssssssyssssssssssshh+                                        ");
        $display("                        :yysssssssssssssssssssssssssssssssssyhysh-     after the reset signal is asserted");
        $display("                      .yyhhdo++oosyyyyssssssssssssssssssssssyyssyh/                                      ");
        $display("                      .dhyh/--------/+oyyyssssssssssssssssssssssssy:   at %4d ps                         ", $time*1000);
        $display("                       .+h/-------------:/osyyysssssssssssssssyyh/.                                      ");
        $display("                        :+------------------::+oossyyyyyyyysso+/s-                                       ");
        $display("                       `s--------------------------::::::::-----:o                                       ");
        $display("                       +:----------------------------------------y`                                      ");
        repeat(5) #(`CYCLE_TIME);
        $finish;
    end
    #(`CYCLE_TIME/2.0) release clk;
     @(negedge clk);
end endtask


endmodule