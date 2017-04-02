//Nelson Campos, March 20, 2017
//copyright 2017 sistenix.com
module top;
  
  logic clock, reset;
  logic [W-1:0] X, Y;
  logic [W_SUM-1:0] mem[ROW_SIZE][ROW_SIZE], sum_array[ROW_SIZE][ROW_SIZE];
  int i, j, pos_x, pos_y, WINDOW_SIZE, A, B, C, S;
  always_comb begin
    mem[0][0:3] = {8'd1,8'd2,8'd3,8'd4};
    mem[1][0:3] = {8'd5,8'd6,8'd7,8'd8};
    mem[2][0:3] = {8'd1,8'd2,8'd3,8'd4};
    mem[3][0:3] = {8'd5,8'd6,8'd7,8'd8};
  end
  
  initial begin
    pos_x = 1;
    pos_y = 0;
    WINDOW_SIZE = 3;
    
    clock = 0;
    reset = 1;
    #15
    reset = 0;
    $display("starting simulation ...");
    //$monitor("X = %d, Y = %d at %d", X, Y, $time);
    for(i=0;i<ROW_SIZE;i++)
      for(j=0; j<ROW_SIZE;j++)begin
        @(posedge clock)begin
         X = mem[i][j];
         sum_array[i][j] = Y;
          $display(X, sum_array[i][j]);
        end
      end
      @(posedge clock)
      sum_array[i][j] = Y;
        
    shift_array();
    
    do_sum(pos_x, pos_y, WINDOW_SIZE);
    $display("%d %d %d %d", A, B, C, S);
    
    $finish;
  end
  
  
  always #5 clock = !clock;
  
  integral_image m_integral_image(.clock(clock), .reset(reset), .new_sample(X), .S(Y));
  
  function shift_array();
    for(int k1 = 0; k1<=ROW_SIZE-1;k1++)begin
      sum_array[k1][0:ROW_SIZE-2] = sum_array[k1][1:ROW_SIZE-1];
      sum_array[k1][ROW_SIZE-1] = sum_array[k1+1][0];
    end
    //sum_array[3][3] = Y;
  endfunction: shift_array
  
  function do_sum(input int pos_x, pos_y, WINDOW_SIZE);
    A = (pos_x >=1 && pos_y >=1)? sum_array[pos_y-1][pos_x-1] : 0;
    B = (pos_y >=1) ? sum_array[pos_x+WINDOW_SIZE-1][pos_y-1] : 0;
    C = (pos_x >= 1) ? sum_array[pos_y+WINDOW_SIZE-1][pos_x-1] : 0;
    S = sum_array[pos_y+WINDOW_SIZE-1][pos_x+WINDOW_SIZE-1]+A-B-C; 
  endfunction: do_sum
  
endmodule: top
