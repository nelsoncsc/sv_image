parameter SIZE_WORD = 8;

parameter HTOT = 10;
parameter MASC = 3;

module window #(parameter SIZE_WORD=8, MASC=3)
               (input  logic clock, reset,
                input logic [SIZE_WORD-1:0] inputPixel,
                output logic [MASC-1:0][SIZE_WORD-1:0]j[MASC-1:0]);
  
  logic [(MASC-1)*SIZE_WORD-1:0] buffer[HTOT-1:0];
  logic [$clog2(HTOT)-1:0] posb;
  
  always_ff @(posedge clock)
    if(reset) begin
      posb <=0;
      for(int k=0; k<MASC; k++)
        for(int i=0; i<MASC; i++)
          j[i][k] <= 0;
    end
    else begin
      j[0][0] <= inputPixel;
      for(int k=0; k<MASC; k++)
        for(int i=1; i<MASC; i++)
          j[i][k] <= j[i-1][k];
      
      buffer[posb] <= j[MASC-1][MASC-2:0];
      
      j[0][MASC-1:1] <= buffer[posb];
      if(posb < HTOT-MASC) posb <= posb + 1;
	     else posb <= 0;
    end
endmodule: window

module sobel #(parameter SIZE_WORD=8)
             (input logic clock,reset,
              input logic [SIZE_WORD-1:0] inputPixel,
              output logic [SIZE_WORD-1:0] outputPixel);
				  
	localparam MASC=3;

	logic [MASC-1:0] [SIZE_WORD-1:0] j [MASC-1:0];
    window #(SIZE_WORD,MASC) my_window(.*);
    
    logic [SIZE_WORD+1:0] gradhi, gradhs, gradvl, gradvr;
	logic [SIZE_WORD+1:0] gradhi_1, gradhs_1, gradvl_1, gradvr_1;

    always_ff @(posedge clock)
     if (reset) begin
        gradhi_1 <= 0;
        gradhs_1 <= 0;
        gradvl_1 <= 0;
        gradvr_1 <= 0;
     end

     else begin
        gradhi_1 <= j[0][0] + j[2][0];
        gradhs_1 <= j[0][2] + j[2][2];
        gradvr_1 <= j[0][0] + j[0][2];
        gradvl_1 <= j[2][0] + j[2][2];
     end

   always_ff @(posedge clock)  
     if (reset) begin
        gradhi <= 0;
        gradhs <= 0;
        gradvl <= 0;
        gradvr <= 0;
     end
     else begin
       gradhi <= gradhi_1 + (j[1][0]<<1);
       gradhs <= gradhs_1 + (j[1][2]<<1);
       gradvr <= gradvl_1 + (j[0][1]<<1);
       gradvl <= gradvr_1 + (j[2][1]<<1);
     end

	  
   logic [SIZE_WORD+1:0] gradv, gradh;
   always_comb begin
      if (gradhi > gradhs) gradh <= gradhi - gradhs;
      else                 gradh <= gradhs - gradhi;
      if (gradvl > gradvr) gradv <= gradvl - gradvr;
      else                 gradv <= gradvr - gradvl;
   end
			
   logic [SIZE_WORD+2:0] grad;
   
   always_comb grad <= gradv + gradh; 
   always_ff @(posedge clock)  
     if (reset) 
        outputPixel <= 0;
     else 
       if (grad[SIZE_WORD+2]) outputPixel <= {SIZE_WORD{1'b1}};
	    else outputPixel <= grad[SIZE_WORD+1:2];
   
endmodule
