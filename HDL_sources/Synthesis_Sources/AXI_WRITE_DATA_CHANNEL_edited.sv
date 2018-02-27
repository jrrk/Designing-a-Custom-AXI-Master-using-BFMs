module AXI_WRITE_DATA_CHANNEL #(data_width=32) (
	input wire		clk,
	input wire		resetn,
	input wire	[data_width - 1:0] 	data,
	input wire		data_valid,
	input wire		go,
	output logic		done,
	input wire		last_transfer,
	output logic		data_sent,
	input wire	[3:0] 	transaction_ID,
	output logic	[3:0] 	WID,
	output logic	[data_width - 1:0] 	WDATA,
	output logic	[(data_width / 8) - 1:0] 	WSTRB,
	output logic		WLAST,
	output logic		WVALID,
	input wire		WREADY,
	output logic	[2:0] 	current_state_out); // 507
/* design AXI_WRITE_DATA_CHANNEL */
/* architecture Behavioral */
typedef enum {FALSE,TRUE} bool_t; // 527
/* VhdBlockSubProgramBody generate_data_width_error: interface not found */
typedef enum [2:0] {reset,
idle,
running,
stalled,
complete} main_fsm_type; // 674
reg width_check; // 612
main_fsm_type current_state, next_state; // 604
reg capture_transaction_ID; // 612
reg clear_transaction_ID; // 612
reg send_data_to_wdata; // 612
assign /*903*/ current_state_out = current_state == reset ? 3'b000 :  current_state == idle ? 3'b001 :  current_state == running ? 3'b010 :  current_state == stalled ? 3'b011 :  current_state == complete ? 3'b100 :  3'b111; // 905
// assign /*432*/ width_check = generate_data_width_error[data_width]; // 434
   assign /*903*/ WDATA = send_data_to_wdata ==  1'b1 ? data : '0;
 // Aggregate 966;
 // 905

always @ ( posedge clk or negedge resetn)
  if (!resetn) /* block const 263 */
WID <= (0<<0);
  else   begin
  if (clear_transaction_ID ==  1'b1)
          begin
      /* block const 263 */
WID <= (0<<0);
      end
          else if     (capture_transaction_ID ==  1'b1)
          begin
      WID <= transaction_ID; // 413
              end
        end
  

always @ ( posedge clk)
  if (resetn ==  1'b0)
  begin
  current_state <= reset; // 413
      end
    else
current_state <= next_state; // 413

always @(current_state or WREADY or go or last_transfer or data_valid)
begin
/* block const 263 */
WSTRB <= (0<<0);
WVALID <=  1'b0; // 413
done <=  1'b0; // 413
capture_transaction_ID <=  1'b0; // 413
clear_transaction_ID <=  1'b0; // 413
data_sent <=  1'b0; // 413
send_data_to_wdata <=  1'b0; // 413
WLAST <=  1'b0; // 413
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
        capture_transaction_ID <=  1'b1; // 413
              end
      
  end
  
  running:
    begin
  next_state <= running; // 413
    send_data_to_wdata <=  1'b1; // 413
    data_sent <= WREADY && data_valid; // 413
    /* block const 263 */
WSTRB <= -1;
WVALID <= data_valid; // 413
    WLAST <= last_transfer; // 413
    if (last_transfer ==  1'b1)
          begin
      if ((WREADY ==  1'b1) && (data_valid ==  1'b1))
                  begin
          next_state <= complete; // 413
            clear_transaction_ID <=  1'b1; // 413
                      end
          
      end
      
  end
  
  complete:
    begin
  done <=  1'b1; // 413
    next_state <= complete; // 413
    if (go ==  1'b0)
          begin
      next_state <= idle; // 413
              end
      
  end
  
  default:
    begin
  next_state <= reset; // 413
      end
  
endcase

end


endmodule // AXI_WRITE_DATA_CHANNEL
