module AXI_READ_DATA_CHANNEL #(data_width=32) (
	input wire		clk,
	input wire		resetn,
	output logic	[data_width - 1:0] 	data,
	output logic		data_valid,
	input wire		fifo_ready,
	output logic		last_transfer,
	input wire		go,
	output logic		done,
	output logic		error,
	output logic	[3:0] 	transaction_ID,
	input wire	[3:0] 	RID,
	input wire	[data_width - 1:0] 	RDATA,
	input wire	[1:0] 	RRESP,
	input wire		RLAST,
	input wire		RVALID,
	output logic		RREADY); // 507
/* design AXI_READ_DATA_CHANNEL */
/* architecture Behavioral */
typedef enum {FALSE,TRUE} bool_t; // 527
/* VhdBlockSubProgramBody generate_data_width_error: interface not found */
typedef enum [2:0] {reset,
idle,
transaction_OKAY,
transaction_ERROR,
complete} main_fsm_type; // 674
reg width_check_OK; // 612
main_fsm_type current_state, next_state; // 604
reg capture_transaction_ID; // 612
reg clear_transaction_ID; // 612
assign /*432*/ width_check_OK = TRUE; // 434
assign /*432*/ data = RDATA; // 434

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
  if (clear_transaction_ID ==  1'b1)
          begin
      /* block const 263 */
transaction_ID <= (0<<3)|(0<<2)|(0<<1)|(0<<0);
      end
          else if     (capture_transaction_ID ==  1'b1)
          begin
      transaction_ID <= RID; // 413
              end
        end
  

always @(current_state or width_check_OK or RVALID or RRESP or RLAST or fifo_ready or go)
begin
RREADY <=  1'b0; // 413
capture_transaction_ID <=  1'b0; // 413
clear_transaction_ID <=  1'b0; // 413
last_transfer <=  1'b0; // 413
error <=  1'b0; // 413
done <=  1'b0; // 413
data_valid <=  1'b0; // 413
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
      capture_transaction_ID <=  1'b1; // 413
        case (width_check_OK)
          TRUE:
                    begin
          next_state <= transaction_OKAY; // 413
                      end
          
          default:
                    begin
          next_state <= transaction_ERROR; // 413
                      end
          
        endcase
      end
      
  end
  
  transaction_OKAY:
    begin
  next_state <= transaction_OKAY; // 413
    if (RVALID ==  1'b1)
          begin
      if (RRESP == 2'b00)
                  begin
          data_valid <=  1'b1; // 413
            if (RLAST ==  1'b1)
                          begin
              last_transfer <=  1'b1; // 413
                next_state <= complete; // 413
                              end
              
          end
           else
          begin
          next_state <= transaction_ERROR; // 413
                      end
                end
      
RREADY <= fifo_ready; // 413
      end
  
  transaction_ERROR:
    begin
  next_state <= transaction_ERROR; // 413
    error <=  1'b1; // 413
    clear_transaction_ID <=  1'b1; // 413
    if (go ==  1'b0)
          begin
      next_state <= idle; // 413
              end
      
  end
  
  complete:
    begin
  next_state <= complete; // 413
    clear_transaction_ID <=  1'b1; // 413
    done <=  1'b1; // 413
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


endmodule // AXI_READ_DATA_CHANNEL
