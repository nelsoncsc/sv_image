`include "RGB2YCbCr.sv"
`include "rgb_if.sv"
`include "ycbcr_if.sv"

parameter N_ADDR = 8;
module ROM(input logic [2:0]address, 
           output logic [23:0]data);
   
   always_comb begin
     case (address)
       00: data <= 0;
       01: data <= (123<<16)+(88<<8)+60;
       02: data <= (100<<16)+(200<<8)+110;
       03: data <= (50<<16)+(50<<8)+10;
       04: data <= (251<<16)+(135<<8)+160;
       05: data <= (185<<16)+(69<<8)+45;
       06: data <= (196<<16)+(188<<8)+201;
       07: data <= (132<<16)+(168<<8)+74;
     endcase
   end
endmodule

module top;
  logic clk;
  logic rst;
  logic [2:0] addr;
  logic [23:0] data;
  int i;
  
  initial begin
    clk = 0;
    rst = 1;
    #22 rst = 0;
     
    for(i = 0; i <= N_ADDR; i++)begin
      @(posedge clk)
      addr <= i;
      in.R <= data[23:16];
      in.G <= data[15:8];
      in.B <= data[7:0];
      $display("R = %d G = %d B = %d", in.R,in.G, in.B);
      @(posedge clk)
      $display("Y = %d Cb = %d Cr = %d", out.Y, out.Cb, out.Cr);
    end
    $finish();
  end
  always #5 clk = !clk;
  
  rgb_if in(clk, rst);
  ycbcr_if out(clk, rst);
  RGB2YCbCr rtl(in, out);
  ROM image(.address(addr), .data(data));
  
endmodule
