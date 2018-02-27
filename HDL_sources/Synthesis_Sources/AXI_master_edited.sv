module AXI_master #(data_width=32) (
	input wire		go,
	output logic		error,
	input wire		RNW,
	output logic		busy,
	output logic		done,
	input wire	[31:0] 	address,
	input wire	[data_width - 1:0] 	write_data,
	output logic	[data_width - 1:0] 	read_data,
	input wire	[7:0] 	burst_length,
	input wire	[6:0] 	burst_size,
	input wire		increment_burst,
	input wire		m_axi_aclk,
	input wire		m_axi_aresetn,
	input wire		m_axi_arready,
	output logic		m_axi_arvalid,
	output logic	[31:0] 	m_axi_araddr,
	output logic	[3:0] 	m_axi_arid,
	output logic	[7:0] 	m_axi_arlen,
	output logic	[2:0] 	m_axi_arsize,
	output logic	[1:0] 	m_axi_arburst,
	output logic		m_axi_arlock,
	output logic	[3:0] 	m_axi_arcache,
	output logic	[2:0] 	m_axi_arprot,
	output logic	[3:0] 	m_axi_arqos,
	output logic	[3:0] 	m_axi_arregion,
	output logic		m_axi_rready,
	input wire		m_axi_rvalid,
	input wire	[data_width - 1:0] 	m_axi_rdata,
	input wire	[1:0] 	m_axi_rresp,
	input wire	[3:0] 	m_axi_rid,
	input wire		m_axi_rlast,
	input wire		m_axi_awready,
	output logic		m_axi_awvalid,
	output logic	[31:0] 	m_axi_awaddr,
	output logic	[3:0] 	m_axi_awid,
	output logic	[7:0] 	m_axi_awlen,
	output logic	[2:0] 	m_axi_awsize,
	output logic	[1:0] 	m_axi_awburst,
	output logic		m_axi_awlock,
	output logic	[3:0] 	m_axi_awcache,
	output logic	[2:0] 	m_axi_awprot,
	output logic	[3:0] 	m_axi_awqos,
	output logic	[3:0] 	m_axi_awregion,
	input wire		m_axi_wready,
	output logic		m_axi_wvalid,
	output logic	[3:0] 	m_axi_wid,
	output logic	[data_width - 1:0] 	m_axi_wdata,
	output logic	[(data_width / 8) - 1:0] 	m_axi_wstrb,
	output logic		m_axi_wlast,
	output logic		m_axi_bready,
	input wire		m_axi_bvalid,
	input wire	[1:0] 	m_axi_bresp,
	input wire	[3:0] 	m_axi_bid,
	output logic		read_data_valid,
	output logic		write_data_valid,
	output logic	[2:0] 	current_state_out,
	output logic	[2:0] 	current_state_rac,
	output logic	[2:0] 	current_state_resp,
	output logic	[2:0] 	current_state_wrdata,
	output logic	[9:0] 	start_out); // 507
/* design AXI_master */
/* architecture Behavioral */
typedef enum {FALSE,TRUE} bool_t; // 527
// Declare component AXI_ADDRESS_CONTROL_CHANNEL; // 950
// Declare component AXI_READ_DATA_CHANNEL; // 963
// Declare component AXI_WRITE_DATA_CHANNEL; // 963
// Declare component AXI_WRITE_DATA_RESPONSE_CHANNEL; // 950
/* VhdBlockSubProgramBody generate_data_width_error: interface not found */
typedef enum [2:0] {reset,
idle,
prepare,
read_transaction,
write_transaction,
error_detected,
complete} main_fsm_type; // 674
reg width_check_OK; // 612
main_fsm_type current_state, next_state; // 908
reg [31:0] transaction_address; // 605
reg start_read_address_transaction; // 612
reg start_read_data_transaction; // 612
reg start_write_address_transaction; // 612
reg start_write_data_transaction; // 612
reg start_write_response_transaction; // 612
reg read_address_transaction_finished; // 612
reg read_data_transaction_finished; // 612
reg write_address_transaction_finished; // 612
reg write_data_transaction_finished; // 612
reg write_response_transaction_finished; // 612
reg read_transaction_error; // 612
reg write_transaction_error; // 612
reg send_write_data; // 612
reg read_data_fifo_enable; // 612
reg write_data_fifo_ready; // 612
reg write_data_fifo_enable; // 612
integer beat_counter; // 900
reg load_beat_counter; // 612
reg enable_beat_counter; // 612
reg write_data_last_transfer; // 612
reg write_data_sent; // 612
reg write_response_error; // 612
reg read_data_last_transfer; // 612
reg read_data_channel_error; // 612
integer captured_burst_length; // 900
integer captured_burst_size; // 900
reg captured_increment_burst; // 612
reg capture_control_signals; // 612
reg [31:0] captured_transaction_address; // 605
reg captured_RNW; // 612
reg [3:0] read_data_transaction_id; // 605
reg [3:0] write_data_transaction_id; // 605
reg [3:0] write_data_response_transaction_id; // 605
integer burst_length_internal; // 900
integer burst_size_internal; // 900
assign /*432*/ width_check_OK = TRUE; // 434
assign /*432*/ write_data_valid = enable_beat_counter; // 434
assign write_data_transaction_id = (0<<3)|(0<<2)|(0<<1)|(0<<0);
assign /*432*/ start_out = {read_address_transaction_finished, write_address_transaction_finished, read_data_transaction_finished, write_data_transaction_finished, write_response_transaction_finished, start_read_address_transaction, start_read_data_transaction, start_write_address_transaction, start_write_data_transaction, start_write_response_transaction}
; // 434
assign /*432*/ burst_length_internal = burst_length + 1; // 434
assign /*432*/ burst_size_internal = burst_size; // 434
assign /*903*/ current_state_out = current_state == reset ? 3'b000 :  current_state == idle ? 3'b001 :  current_state == prepare ? 3'b010 :  current_state == read_transaction ? 3'b011 :  current_state == write_transaction ? 3'b100 :  current_state == error_detected ? 3'b101 :  current_state == complete ? 3'b110 :  3'b111; // 905

always @ ( posedge m_axi_aclk)
  if (m_axi_aresetn ==  1'b0)
  begin
  current_state <= reset; // 413
      end
    else
current_state <= next_state; // 413

always @ ( posedge m_axi_aclk)
if (m_axi_aresetn ==  1'b0)
    begin
  beat_counter <= 0; // 413
      end
  else if (load_beat_counter ==  1'b1)
          begin
      beat_counter <= burst_length_internal - 1; // 413
              end
      
else if (enable_beat_counter ==  1'b1)
          begin
      if (beat_counter > 0)
                  begin
          beat_counter <= beat_counter - 1; // 413
                      end
          
      end
      
always @ ( posedge m_axi_aclk)
if (m_axi_aresetn ==  1'b0)
  begin
  captured_burst_length <= 0; // 413
    captured_burst_size <= 0; // 413
    captured_increment_burst <=  1'b0; // 413
    /* block const 263 */
captured_transaction_address <= (0<<31)|(0<<30)|(0<<29)|(0<<28)|(0<<27)|(0<<26)|(0<<25)|(0<<24)|(0<<23)|(0<<22)|(0<<21)|(0<<20)|(0<<19)|(0<<18)|(0<<17)|(0<<16)|(0<<15)|(0<<14)|(0<<13)|(0<<12)|(0<<11)|(0<<10)|(0<<9)|(0<<8)|(0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0);
captured_RNW <=  1'b0; // 413
      end
  else if (capture_control_signals ==  1'b1)
  begin
  captured_burst_length <= burst_length_internal; // 413
    captured_burst_size <= burst_size_internal; // 413
    captured_increment_burst <= increment_burst; // 413
    captured_transaction_address <= transaction_address; // 413
    captured_RNW <= RNW; // 413
      end
  
always @(width_check_OK or current_state or read_address_transaction_finished or read_data_transaction_finished or write_address_transaction_finished or write_data_transaction_finished or write_response_transaction_finished or go or captured_RNW or address or write_data_sent or write_response_error or beat_counter or write_data_sent)
begin
transaction_address <= address; // 413
start_read_address_transaction <=  1'b0; // 413
start_read_data_transaction <=  1'b0; // 413
start_write_address_transaction <=  1'b0; // 413
start_write_data_transaction <=  1'b0; // 413
start_write_response_transaction <=  1'b0; // 413
busy <=  1'b1; // 413
done <=  1'b0; // 413
error <=  1'b0; // 413
load_beat_counter <=  1'b0; // 413
send_write_data <=  1'b0; // 413
capture_control_signals <=  1'b0; // 413
write_data_last_transfer <=  1'b0; // 413
enable_beat_counter <=  1'b0; // 413
case (current_state)
  reset:
    begin
  next_state <= idle; // 413
      end
  
  idle:
    begin
  next_state <= idle; // 413
    busy <=  1'b0; // 413
    if (go ==  1'b1)
          begin
      case (width_check_OK)
          TRUE:
                    begin
          capture_control_signals <=  1'b1; // 413
            next_state <= prepare; // 413
                      end
          
          default:
                    begin
          next_state <= error_detected; // 413
                      end
          
        endcase
      end
      
  end
  
  prepare:
    begin
  load_beat_counter <=  1'b1; // 413
    busy <=  1'b0; // 413
    case (captured_RNW)
       1'b1:
            begin
      next_state <= read_transaction; // 413
              end
      
       1'b0:
            begin
      next_state <= write_transaction; // 413
              end
      
      default:
            begin
      next_state <= error_detected; // 413
              end
      
    endcase
  end
  
  read_transaction:
    begin
  next_state <= read_transaction; // 413
    start_read_address_transaction <=  1'b1; // 413
    enable_beat_counter <= write_data_sent; // 413
    if (read_address_transaction_finished ==  1'b1)
          begin
      start_read_data_transaction <=  1'b1; // 413
              end
      
if (read_data_transaction_finished ==  1'b1)
          begin
      next_state <= complete; // 413
              end
      
  end
  
  write_transaction:
    begin
  next_state <= write_transaction; // 413
    start_write_address_transaction <=  1'b1; // 413
    enable_beat_counter <= write_data_sent; // 413
    if (write_address_transaction_finished ==  1'b1)
          begin
      start_write_data_transaction <=  1'b1; // 413
              end
      
if (write_data_transaction_finished ==  1'b1)
          begin
      start_write_response_transaction <=  1'b1; // 413
              end
      
if (beat_counter < 1)
          begin
      write_data_last_transfer <=  1'b1; // 413
              end
       else
      begin
      write_data_last_transfer <=  1'b0; // 413
              end
      if ((write_address_transaction_finished ==  1'b1 && write_data_transaction_finished ==  1'b1) && write_response_transaction_finished ==  1'b1)
          begin
      if (write_response_error ==  1'b1)
                  begin
          next_state <= error_detected; // 413
                      end
           else
          begin
          next_state <= complete; // 413
                      end
                end
      
  end
  
  error_detected:
    begin
  next_state <= error_detected; // 413
    done <=  1'b1; // 413
    error <=  1'b1; // 413
    if (go ==  1'b0)
          begin
      next_state <= idle; // 413
              end
      
  end
  
  complete:
    begin
  next_state <= complete; // 413
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

AXI_ADDRESS_CONTROL_CHANNEL read_address_channel (
	.clk(m_axi_aclk),
	.resetn(m_axi_aresetn),
	.go(start_read_address_transaction),
	.done(read_address_transaction_finished),
	.error(read_transaction_error),
	.address(captured_transaction_address),
	.burst_length(captured_burst_length),
	.burst_size(captured_burst_size),
	.increment_burst(captured_increment_burst),
	.AxADDR(m_axi_araddr),
	.AxVALID(m_axi_arvalid),
	.AxREADY(m_axi_arready),
	.AxID(m_axi_arid),
	.AxLEN(m_axi_arlen),
	.AxSIZE(m_axi_arsize),
	.AxBURST(m_axi_arburst),
	.AxLOCK(m_axi_arlock),
	.AxCACHE(m_axi_arcache),
	.AxPROT(m_axi_arprot),
	.AxQOS(m_axi_arqos),
	.AxREGION(m_axi_arregion),
	.current_state_out(current_state_rac)); // 879
AXI_ADDRESS_CONTROL_CHANNEL write_address_channel (
	.clk(m_axi_aclk),
	.resetn(m_axi_aresetn),
	.go(start_write_address_transaction),
	.done(write_address_transaction_finished),
	.error(write_transaction_error),
	.address(captured_transaction_address),
	.burst_length(captured_burst_length),
	.burst_size(captured_burst_size),
	.increment_burst(captured_increment_burst),
	.AxADDR(m_axi_awaddr),
	.AxVALID(m_axi_awvalid),
	.AxREADY(m_axi_awready),
	.AxID(m_axi_awid),
	.AxLEN(m_axi_awlen),
	.AxSIZE(m_axi_awsize),
	.AxBURST(m_axi_awburst),
	.AxLOCK(m_axi_awlock),
	.AxCACHE(m_axi_awcache),
	.AxPROT(m_axi_awprot),
	.AxQOS(m_axi_awqos),
	.AxREGION(m_axi_awregion), // 879
	.current_state_out()); // 879
AXI_READ_DATA_CHANNEL #(.data_width(data_width)) read_data_channel (
	.clk(m_axi_aclk),
	.resetn(m_axi_aresetn),
	.go(start_read_data_transaction),
	.done(read_data_transaction_finished),
	.error(read_data_channel_error),
	.data(read_data),
	.data_valid(read_data_valid),
	.fifo_ready( 1'b1),
	.last_transfer(read_data_last_transfer),
	.transaction_ID(read_data_transaction_id),
	.RDATA(m_axi_rdata),
	.RRESP(m_axi_rresp),
	.RVALID(m_axi_rvalid),
	.RREADY(m_axi_rready),
	.RID(m_axi_rid),
	.RLAST(m_axi_rlast)); // 879
AXI_WRITE_DATA_CHANNEL #(.data_width(data_width)) write_data_channel (
	.clk(m_axi_aclk),
	.resetn(m_axi_aresetn),
	.go(start_write_data_transaction),
	.done(write_data_transaction_finished),
	.data(write_data),
	.data_valid( 1'b1),
	.last_transfer(write_data_last_transfer),
	.data_sent(write_data_sent),
	.transaction_ID(write_data_transaction_id),
	.WID(m_axi_wid),
	.WDATA(m_axi_wdata),
	.WSTRB(m_axi_wstrb),
	.WVALID(m_axi_wvalid),
	.WREADY(m_axi_wready),
	.WLAST(m_axi_wlast),
	.current_state_out(current_state_wrdata)); // 879
AXI_WRITE_DATA_RESPONSE_CHANNEL write_data_response_channel (
	.clk(m_axi_aclk),
	.resetn(m_axi_aresetn),
	.go(start_write_response_transaction),
	.error(write_response_error),
	.done(write_response_transaction_finished),
	.transaction_ID(write_data_response_transaction_id),
	.BRESP(m_axi_bresp),
	.BVALID(m_axi_bvalid),
	.BREADY(m_axi_bready),
	.BID(m_axi_bid),
	.current_state_out(current_state_resp)); // 879

endmodule
   
