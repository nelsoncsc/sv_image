`include "sobel.sv"

module top();
  logic clock, reset;
  logic [4:0] i, k;
  logic [SIZE_WORD-1:0] inputPixel;
  logic [SIZE_WORD-1:0] outputPixel;
  
  initial begin
    $display("testing ...");
    clock = 0;
    $monitor("%d %d %d", i, inputPixel, outputPixel);
    for(i=0; i<31; i++)begin
        #0 @(posedge clock)
        inputPixel=$random;
      end
    #100;
    $finish;
  end
  always #5 clock = !clock;
  
  sobel filter(.clock(clock), .reset(1'b0), .inputPixel(inputPixel),
               .outputPixel(outputPixel));
endmodule: top
