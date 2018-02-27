module AXI_ADDRESS_CONTROL_CHANNEL(
	input wire		clk,
	input wire		resetn,
	input wire		go,
	output logic		done,
	output logic		error,
	input wire	[31:0] 	address,
	input integer           burst_length,
	input integer           burst_size,
	input wire		increment_burst,
	output logic	[3:0] 	AxID,
	output logic	[31:0] 	AxADDR,
	output logic	[7:0] 	AxLEN,
	output logic	[2:0] 	AxSIZE,
	output logic	[1:0] 	AxBURST,
	output logic		AxLOCK,
	output logic	[3:0] 	AxCACHE,
	output logic	[2:0] 	AxPROT,
	output logic		AxVALID,
	input wire		AxREADY,
	output logic	[3:0] 	AxQOS,
	output logic	[3:0] 	AxREGION,
	output logic	[2:0] 	current_state_out);
/* design AXI_ADDRESS_CONTROL_CHANNEL */
/* architecture Behavioral */
typedef enum {FALSE,TRUE} bool_t;
typedef enum [2:0] {reset,
idle,
running,
error_detected,
complete} main_fsm_type;
main_fsm_type current_state, next_state; // 604
reg address_enable; // 612

always @ ( posedge clk)
  if (resetn ==  1'b0)
  begin
  current_state <= reset;
      end
    else
current_state <= next_state;
assign /*903*/ AxADDR = address_enable ==  1'b1 ? address : 'b0; // 978
assign AxID = (0<<0);
assign /*432*/ AxLOCK =  1'b0; // 434
assign AxCACHE = (0<<0);
assign AxPROT = (0<<0);
assign AxQOS = (0<<0);
assign AxREGION = (0<<0);
assign /*903*/ current_state_out = current_state == reset ? 3'b000 :  current_state == idle ? 3'b001 :  current_state == running ? 3'b010 :  current_state == error_detected ? 3'b011 :  current_state == complete ? 3'b100 :  3'b111; // 905

always @(current_state or burst_size)
begin
if ((current_state == running))
  begin
  case (burst_size)
      1:
            begin
      AxSIZE <= 3'b000; // 413
              end
      
      2:
            begin
      AxSIZE <= 3'b001; // 413
              end
      
      4:
            begin
      AxSIZE <= 3'b010; // 413
              end
      
      8:
            begin
      AxSIZE <= 3'b011; // 413
              end
      
      16:
            begin
      AxSIZE <= 3'b100; // 413
              end
      
      32:
            begin
      AxSIZE <= 3'b101; // 413
              end
      
      64:
            begin
      AxSIZE <= 3'b110; // 413
              end
      
      128:
            begin
      AxSIZE <= 3'b111; // 413
              end
      
      default:
            begin
      AxSIZE <= 'bxxx; // 413
              end
      
    endcase
  end
   else
  begin
  AxSIZE <= 'bxxx; // 413
      end
  
end


always @(current_state or increment_burst)
begin
if ((current_state == running))
  begin
  case (increment_burst)
       1'b1:
            begin
      AxBURST <= 2'b01; // 413
              end
      
       1'b0:
            begin
      AxBURST <= 2'b00; // 413
              end
      
      default:
            begin
      begin end      end
      
    endcase
  end
   else
  begin
  AxBURST <= 'bxx; // 413
      end
  
end


always @(current_state or burst_length)
begin
if ((current_state == running))
  begin
  AxLEN <= (burst_length - 1); // 427
      end
   else
  begin
  /* block const 263 */
AxLEN <= (-3<<0);
  end
  
end


always @(current_state or go or AxREADY or burst_size or burst_length)
begin
done <=  1'b0;
address_enable <=  1'b0;
AxVALID <=  1'b0;
error <=  1'b0;
case (current_state)
  reset:
    begin
  next_state <= idle;
      end
  
  idle:
    begin
  next_state <= idle;
    if (go ==  1'b1)
          begin
      case (burst_size)
          1, 2, 4, 8, 16, 32, 64, 128:
                    begin
          if (burst_length >= 1 && burst_length <= 256)
                            begin
              next_state <= running;
                              end
              else
                            begin
              next_state <= error_detected;
                              end
          end
          
          default:
                    begin
          next_state <= error_detected;
                      end
          
        endcase
      end
      
  end
  
  running:
    begin
  next_state <= running;
    address_enable <=  1'b1;
    AxVALID <=  1'b1;
    if (AxREADY ==  1'b1)
          begin
      next_state <= complete;
              end
      
  end
  
  complete:
    begin
  next_state <= complete;
    done <=  1'b1;
    if (go ==  1'b0)
          begin
      next_state <= idle;
              end
      
  end
  
  error_detected:
    begin
  next_state <= error_detected;
    done <=  1'b1;
    error <=  1'b1;
    if (go ==  1'b0)
          begin
      next_state <= idle;
              end
      
  end
  
  default:
    begin
  next_state <= reset;
      end
  
endcase

end


endmodule // AXI_ADDRESS_CONTROL_CHANNEL
