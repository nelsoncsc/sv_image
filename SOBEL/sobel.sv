parameter WORD_SIZE = 8;

parameter ROW_SIZE = 10;
parameter BUFFER_SIZE = 3;

module window #(parameter WORD_SIZE=8, BUFFER_SIZE=3)
               (input  logic clock, reset,
                input logic [WORD_SIZE-1:0] inputPixel,
                output logic [BUFFER_SIZE-1:0][WORD_SIZE-1:0]j[BUFFER_SIZE-1:0]);
  
  logic [(BUFFER_SIZE-1)*WORD_SIZE-1:0] buffer[ROW_SIZE-1:0];
  logic [$clog2(ROW_SIZE)-1:0] posb;
  
  always_ff @(posedge clock)
    if(reset) begin
      posb <=0;
      for(int k=0; k<BUFFER_SIZE; k++)
        for(int i=0; i<BUFFER_SIZE; i++)
          j[i][k] <= 0;
    end
    else begin
      j[0][0] <= inputPixel;
      for(int k=0; k<BUFFER_SIZE; k++)
        for(int i=1; i<BUFFER_SIZE; i++)
          j[i][k] <= j[i-1][k];
      
      buffer[posb] <= j[BUFFER_SIZE-1][BUFFER_SIZE-2:0];
      
      j[0][BUFFER_SIZE-1:1] <= buffer[posb];
      if(posb < ROW_SIZE-BUFFER_SIZE) posb <= posb + 1;
	     else posb <= 0;
    end
endmodule: window

module sobel #(parameter WORD_SIZE=8)
             (input logic clock,reset,
              input logic [WORD_SIZE-1:0] inputPixel,
              output logic [WORD_SIZE-1:0] outputPixel);
				  
	localparam BUFFER_SIZE=3;

  logic [BUFFER_SIZE-1:0] [WORD_SIZE-1:0] j [BUFFER_SIZE-1:0];
  window #(WORD_SIZE,BUFFER_SIZE) my_window(.*);
    
  logic [WORD_SIZE+1:0] gradhi, gradhs, gradvl, gradvr;
	
   always_ff @(posedge clock)  
     if (reset) begin
        gradhi <= 0;
        gradhs <= 0;
        gradvl <= 0;
        gradvr <= 0;
     end
     else begin
       gradhi <= j[0][0] + j[2][0] + (j[1][0]<<1);
       gradhs <= j[0][2] + j[2][2] + (j[1][2]<<1);
       gradvr <= j[2][0] + j[2][2] + (j[0][1]<<1);
       gradvl <= j[0][0] + j[0][2] + (j[2][1]<<1);
     end

	  
   logic [WORD_SIZE+1:0] gradv, gradh;
   always_comb begin
      if (gradhi > gradhs) gradh <= gradhi - gradhs;
      else                 gradh <= gradhs - gradhi;
      if (gradvl > gradvr) gradv <= gradvl - gradvr;
      else                 gradv <= gradvr - gradvl;
   end
			
   logic [WORD_SIZE+2:0] grad;
   
   always_comb grad <= gradv + gradh; 
   always_ff @(posedge clock)  
     if (reset) 
        outputPixel <= 0;
     else 
       if (grad[WORD_SIZE+2]) outputPixel <= {WORD_SIZE{1'b1}};
	    else outputPixel <= grad[WORD_SIZE+1:2];
   
endmodule
