parameter W = 8;
parameter BUFFER_SIZE = 2;
parameter ROW_SIZE = 4;
parameter W_SUM = $clog2(ROW_SIZE*ROW_SIZE*(1<<W));

module integral_image(input logic clock, reset,
                      input logic [W-1:0]new_sample,
                      output logic [W_SUM-1:0] S);

  logic [W_SUM-1:0] s[BUFFER_SIZE][BUFFER_SIZE];
  logic [W-1:0] prevS;
  logic [W_SUM-1:0] buffer[ROW_SIZE];
  logic [$clog2(ROW_SIZE)-1:0] ptr, next_ptr;
  logic [$clog2(ROW_SIZE):0] row_counter;
  
  always_comb begin
    if(reset) S <= 0;
    else S <= prevS+s[1][0]+s[0][1]-s[0][0];
  end
  
  always_comb begin
    if(ptr+1 < ROW_SIZE) next_ptr <= ptr+1;
    else next_ptr <= 0;
  end
  
  always_ff @(posedge clock) begin
    if(reset)begin
      ptr <= 0;
      row_counter <= 0;
      s[0][0] <= 0;
      s[0][1] <= 0;
      s[1][0] <= 0;
      s[1][1] <= 0;
      for(int i = 0; i < ROW_SIZE; i++)
        buffer[i] <= 0;
    end
    else begin
      prevS <= new_sample;
      s[1][0] <= S;
      buffer[ptr] <=  S;
      s[0][1] <= buffer[next_ptr];
      s[0][0] <= s[0][1];
	  ptr <= next_ptr;
      if (ptr == 0) begin
        if(row_counter < ROW_SIZE)
          row_counter <= row_counter+1;
        else row_counter <= 0;
        
        s[0][0] <= '0;
        s[1][0] <= '0;
      end
    end
  end

  initial begin
    $monitor("dut => new_sample = %d prevS = %d s[1][0] = %d s[0][1] = %d s[0][0] = %d buffer[ptr] = %d S = %d at time = %d (ptr=%d, next_ptr = %d, row_counter = %d)", new_sample, prevS, s[1][0], s[0][1], s[0][0], buffer[ptr], S, $time, ptr, next_ptr, row_counter);
  end

endmodule: integral_image
