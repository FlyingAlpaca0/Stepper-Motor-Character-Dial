// EEM16 - Logic Design
// 2018_05_01
// Design Assignment #3 
// dassign3.v

// 
// Motor Controller 
// Produces the proper motor_drv pulse sequence
// Sample code from Design Assignment #2 for your reference. 
// You can choose to modify this to your design if you so choose.
// 
module motor_ctrl (motor_drv, done, forw, rev, reset, drv_clk);
   output [3:0] motor_drv;
   output 	done;
   input 	forw, rev, reset, drv_clk;

   //
   // Parameters declaration
   //
   parameter P0_ST = 2'b00;
   parameter P1_ST = 2'b01;
   parameter P2_ST = 2'b10;
   parameter P3_ST = 2'b11;

   reg [1:0] 	state, nx_state;
   reg 		done;
   reg [3:0] 	motor_drv;

   always @(posedge drv_clk) begin
      state <= nx_state;
   end

   always @(state or forw or rev or reset) begin
      if (reset) begin
         nx_state = P0_ST;
	 motor_drv = 4'b0000;
	 done = 0;
      end
      else begin
         if (forw == 1'b1) begin
            case (state)
              P0_ST: begin
                 nx_state = P1_ST;
		 motor_drv = 4'b0010;
		 done = 1'b1;
              end
              P1_ST: begin
          	 nx_state = P2_ST;
		 motor_drv = 4'b0100;
		 done = 1'b0;
    	      end
              P2_ST: begin
                 nx_state = P3_ST;
		 motor_drv = 4'b1000;
		 done = 1'b0;
              end
              P3_ST: begin
                 nx_state = P0_ST;
		 motor_drv = 4'b0001;
		 done = 1'b0;
              end
              default: begin
              	 nx_state = P0_ST;
            	 motor_drv = 4'b0000;
            	 done = 1'b0;
              end
            endcase // case (state)
         end // if (forw == 1'b1)
         else if (rev == 1'b1) begin
            case (state)
              P0_ST: begin
                 nx_state = P3_ST;
		 motor_drv = 4'b1000;
		 done = 1'b1;
              end
              P1_ST: begin
                 nx_state = P0_ST;
		 motor_drv = 4'b0001;
		 done = 1'b0;
              end
              P2_ST: begin
                 nx_state = P1_ST;
		 motor_drv = 4'b0010;
		 done = 1'b0;
              end
              P3_ST: begin
                 nx_state = P2_ST;
		 motor_drv = 4'b0100;
		 done = 1'b0;
              end
              default: begin
              	 nx_state = P0_ST;
            	 motor_drv = 4'b0000;
            	 done = 1'b0;
              end
            endcase // case (state)
         end // if (rev == 1'b1)
         else begin
            nx_state = state;
            motor_drv = 4'b0000;
            done = 1'b0;
         end
      end // else: !if(reset)
   end // always @ (state or forw or rev or reset)
endmodule
// 
// ASCII to Letter Dial Position 
// Converts the 7-bit ASCII to the proper position location from 0-31. 
// Sample code from Design Assignment #1 for your reference. 
// You can choose to modify this to your design if you so choose.
// 
module ascii2pos(pos, ascii);
   output [4:0] pos;

   reg [4:0] 	pos;
   
   input [6:0] 	ascii;

   always @(ascii) begin
      case (ascii)
	7'b010_0000:
	  pos[4:0] = 5'b00000;
	7'b110_0001:
	  pos[4:0] = 5'b00001;
	7'b110_0010:
	  pos[4:0] = 5'b00010;
	7'b110_0011:
	  pos[4:0] = 5'b00011;
	7'b110_0100:
	  pos[4:0] = 5'b00100;
	7'b110_0101:
	  pos[4:0] = 5'b00101;
	7'b110_0110:
	  pos[4:0] = 5'b00110;
	7'b110_0111:
	  pos[4:0] = 5'b00111;
	7'b110_1000:
	  pos[4:0] = 5'b01000;
	7'b110_1001:
	  pos[4:0] = 5'b01001;
	7'b110_1010:
	  pos[4:0] = 5'b01010;
	7'b110_1011:
	  pos[4:0] = 5'b01011;
	7'b110_1100:
	  pos[4:0] = 5'b01100;
	7'b110_1101:
	  pos[4:0] = 5'b01101;
	7'b110_1110:
	  pos[4:0] = 5'b01110;
	7'b110_1111:
	  pos[4:0] = 5'b01111;
	7'b111_0000:
	  pos[4:0] = 5'b10000;
	7'b111_0001:
	  pos[4:0] = 5'b10001;
	7'b111_0010:
	  pos[4:0] = 5'b10010;
	7'b111_0011:
	  pos[4:0] = 5'b10011;
	7'b111_0100:
	  pos[4:0] = 5'b10100;
	7'b111_0101:
	  pos[4:0] = 5'b10101;
	7'b111_0110:
	  pos[4:0] = 5'b10110;
	7'b111_0111:
	  pos[4:0] = 5'b10111;
	7'b111_1000:
	  pos[4:0] = 5'b11000;
	7'b111_1001:
	  pos[4:0] = 5'b11001;
	7'b111_1010:
	  pos[4:0] = 5'b11010;
	7'b010_1100:
	  pos[4:0] = 5'b11101;
	7'b010_1110:
	  pos[4:0] = 5'b11110;
	7'b011_1111:
	  pos[4:0] = 5'b11111;
	default:
	  pos[4:0] = 5'b00000;
      endcase // case (ascii)
   end // always @ (ascii)
endmodule 

//
// vvv--- Add any modules you need below---vvv
//
module motor_steps (mtr_steps, mtr_pos, rev);
  output [2:0] mtr_steps;
  input [4:0] mtr_pos;
  input rev;
  
  reg [2:0] mtr_steps;
  
  always @ (mtr_pos or rev) begin
    if(rev) begin
      casez(mtr_pos)
        5'bzzz00: mtr_steps = 3'd7;
        default:  mtr_steps = 3'd6;
      endcase
    end else begin
      casez(mtr_pos)
        5'bzzz11: mtr_steps = 3'd7;
        default:  mtr_steps = 3'd6;
      endcase
    end
  end
endmodule
//
// Main Module
//
module dassign3(motor, led, btnC, sw, clk, btnU);
   output [3:0]     motor;
   output [15:0] 		led;
   input 			btnC;
   input [6:0] 			sw;
   input 			clk, btnU;
   
   wire sys_clk;
   assign sys_clk = clk;
   
   wire [6:0] 			ascii_in;
   assign ascii_in = sw;
   
   reg 			ready;
   assign led[15] = ready;
   wire [3:0] 		motor_drv;
   wire [3:0]       motor;
   assign led[3:0] = motor_drv;
   assign motor = motor_drv;
   
   wire req;
   assign req = btnC;
   
   wire reset;
   assign reset = btnU;
   //
   // Instantiate the motor clock divider
   //
   wire			motor_clk;   

   clock_div	mclkdiv(sys_clk, reset, motor_clk);
   //assign motor_clk = sys_clk;
   //
   // Instantiate the motor control
   //
   wire			motor_0001;
   reg			motor_forw, motor_rev;
   motor_ctrl 		mctl(motor_drv, motor_0001, motor_forw, motor_rev, reset, motor_clk);

   //
   // Instantiate the ascii-to-position mapping
   //
   wire [4:0] 		pos;
   ascii2pos		a2p(pos, ascii_in);    

   //
   // Instantiate any additional modules you need
   //
  wire [2:0] steps;
  motor_steps mtrstep(steps, curr_pos, motor_rev);
   //
   // Declare your variables
   //
   parameter SST0 = 2'b00;
   parameter SST1 = 2'b01;
   parameter SST2 = 2'b10;
   parameter SST3 = 2'b11;
  reg [1:0] state, next_state;
  //reg ready;
  reg done_moving;
  reg [2:0] count;
  //reg motor_forw, motor_rev;
  reg [4:0] curr_pos;
  reg [4:0] required_pos;
  reg move_forw;
  reg move_rev;
  
  assign led[14] = req;
  assign led[12] = move_forw;
  assign led[11] = motor_forw;
  assign led[10] = move_rev;
  assign led[9] = motor_rev;
  assign led[8] = done_moving;
  assign led[7:5] = count;
  
  //assign led[13:9] = curr_pos;
  //assign led[8:4] = required_pos;
  
   //
   // Your code start below
   //
   
   always @(posedge sys_clk) begin
      //
      // Storage Elements
      //  
     state <= next_state;
   end

   always @(*) begin
         //
         // Combinational Logic Elements
         //
        if(reset) begin
          next_state = SST0;
          required_pos = 5'd0;
          ready = 1'b0;
          move_forw = 1'b0;
          move_rev = 1'b0;
        end else begin
          case(state)
            SST0: begin
              //move_forw = 1'b0;
              //move_rev = 1'b0;
              if(req) begin
                required_pos = pos;
                next_state = SST2;
                ready = 1'b0;
              end else begin
                next_state = state;
                ready = 1'b1;
              end
            end
            //SST1: begin
              //ready = 1'b0;
              //motor_forw = 1'b1;
              
              //if(done_moving == 1'b1) begin
                //move_forw = 1'b0;
                //move_rev = 1'b0;
                //next_state = SST2;
              //end else begin
                //next_state = state;
              //end
            //end
             SST2: begin
               //ready = 1'b0;
               //done_moving = 1'b0;
               
               if(curr_pos == required_pos) begin
                 //motor_forw = 1'b0;
                 next_state = SST0;
               end else begin
                  if(done_moving == 1'b0) begin
                     if (required_pos > curr_pos) begin
                                         if((required_pos - curr_pos) <= 16) begin
                                           move_forw = 1'b1;
                                           move_rev = 1'b0;
                                        end else begin
                                           move_rev = 1'b1;
                                           move_forw = 1'b0;
                                        end
                     end else begin
                                         if((curr_pos - required_pos) <= 16) begin
                                           move_rev = 1'b1;
                                           move_forw = 1'b0;
                                        end else begin
                                           move_forw = 1'b1;
                                           move_rev = 1'b0;
                                        end
                     end 
                     next_state = SST2;
                 end else begin
                     move_forw = 1'b0;
                     move_rev = 1'b0;
                     next_state = SST2;
                 end
               end
              end
            default: begin
              ready = 1'b1;
              next_state = SST0;
            end
          endcase
        end
      end
     
     always @(posedge motor_clk or posedge reset) begin
          if(reset) begin
                 curr_pos <= 5'd0;
                 motor_forw <= 1'b0;
                 motor_rev <= 1'b0;
                 done_moving <= 1'b0;
                 count <= 3'd0;
          end else if(move_forw || move_rev) begin
             if(count != steps) begin
             done_moving <= 1'b0;
                if(move_forw && !move_rev) begin
                    motor_forw <= 1'b1;
                    motor_rev <= 1'b0;
                end else if (!move_forw && move_rev) begin
                    motor_rev <= 1'b1;
                    motor_forw <= 1'b0;
                end else begin
                     motor_forw <= 1'b0;
                     motor_rev <= 1'b0;
                end
                count <= count + 1;
                      
            end else begin
            
            
             count <= 3'd0;
                       if(motor_forw == 1'b1) begin
                         if(curr_pos == 5'b11111)
                              curr_pos <= 5'b00000;
                         else
                              curr_pos <= curr_pos + 1;
                       end else begin
                         if(curr_pos == 5'b00000)
                              curr_pos <= 5'b11111;
                         else
                              curr_pos <= curr_pos - 1;
                       end
                       done_moving <= 1'b1;
                       motor_forw <= 1'b0;
                       motor_rev <= 1'b0;
           end
        end else begin
          motor_forw <= 1'b0;
          motor_rev <= 1'b0;
          count <= 1'b0;
          done_moving <= 1'b0;
        end
      end
   endmodule
       
       

    
