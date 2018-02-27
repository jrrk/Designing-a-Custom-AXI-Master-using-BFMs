module AXI_WRITE_DATA_RESPONSE_CHANNEL(
	input wire		clk,
	input wire		resetn,
	input wire		go,
	output logic		done,
	output logic		error,
	output logic	[3:0] 	transaction_ID,
	input wire	[3:0] 	BID,
	input wire	[1:0] 	BRESP,
	input wire		BVALID,
	output logic		BREADY,
	output logic	[2:0] 	current_state_out); // 507
/* design AXI_WRITE_DATA_RESPONSE_CHANNEL */
/* architecture Behavioral */
typedef enum {FALSE,TRUE} bool_t; // 527
typedef enum [2:0] {reset,
idle,
running,
success,
error_detected,
complete} main_fsm_type; // 674
main_fsm_type current_state, next_state; // 908
assign /*903*/ current_state_out = current_state == reset ? 3'b000 :  current_state == idle ? 3'b001 :  current_state == running ? 3'b010 :  current_state == success ? 3'b011 :  current_state == error_detected ? 3'b100 :  current_state == complete ? 3'b101 :  3'b111; // 905

always @ ( posedge clk)
  if (resetn ==  1'b0)
  begin
  current_state <= reset; // 413
      end
    else
current_state <= next_state; // 413

always @ ( posedge clk or negedge resetn)
  if (!resetn) /* block const 263 */
transaction_ID <= (0<<3)|(0<<2)|(0<<1)|(0<<0);
  else   begin
  transaction_ID <= BID; // 413
      end
  

always @(current_state or BRESP or BVALID or go)
begin
BREADY <=  1'b0; // 413
error <=  1'b0; // 413
done <=  1'b0; // 413
case (current_state)
  reset:
    begin
  next_state <= idle; // 413
      end
  
  idle:
    begin
  next_state <= idle; // 413
    if (go ==  1'b1)
          begin
      next_state <= running; // 413
              end
      
  end
  
  running:
    begin
  next_state <= running; // 413
    BREADY <=  1'b1; // 413
    if (BVALID ==  1'b1)
          begin
      if (BRESP == 2'b00)
                  begin
          next_state <= success; // 413
                      end
           else
          begin
          next_state <= error_detected; // 413
                      end
                end
      
  end
  
  success:
    begin
  done <=  1'b1; // 413
    next_state <= success; // 413
    if (BVALID ==  1'b0)
          begin
      if (go ==  1'b0)
                  begin
          next_state <= idle; // 413
                      end
          
      end
      
  end
  
  error_detected:
    begin
  done <=  1'b1; // 413
    error <=  1'b1; // 413
    BREADY <=  1'b0; // 413
    next_state <= error_detected; // 413
    if (BVALID ==  1'b0)
          begin
      if (go ==  1'b0)
                  begin
          next_state <= idle; // 413
                      end
          
      end
      
  end
  
  default:
    begin
  BREADY <=  1'b0; // 413
    next_state <= reset; // 413
      end
  
endcase

end


endmodule
