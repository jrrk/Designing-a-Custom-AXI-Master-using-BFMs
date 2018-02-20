// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// import ariane_pkg::*;

module wrap
#(
    parameter NB_CORES             = 1,
    parameter AXI_ADDR_WIDTH       = 32,
    parameter AXI_DATA_WIDTH       = 64,
    parameter AXI_ID_MASTER_WIDTH  = 10,
    parameter AXI_ID_SLAVE_WIDTH   = 10,
    parameter AXI_USER_WIDTH       = 0
  )
(
    // Clock and Reset
    input logic       clk,
    input logic       rst_top,
    output wire       CA,
    output wire       CB,
    output wire       CC,
    output wire       CD,
    output wire       CE,
    output wire       CF,
    output wire       CG,
    output wire       DP,
    output wire [7:0] AN
  );

  assign rst = !rst_top;
  
    AXI_BUS #(
              .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH    ),
              .AXI_DATA_WIDTH ( AXI_DATA_WIDTH    ),
              .AXI_ID_WIDTH   ( AXI_ID_MASTER_WIDTH  ),
              .AXI_USER_WIDTH ( AXI_USER_WIDTH    )
    ) dbg_master();
    
wire go;   
wire error;
wire RNW = ~WREN;  
wire busy; 
wire done; 
wire request_to_go;
wire [31:0] address = ADDR;   
wire [AXI_DATA_WIDTH-1:0] write_data = TO_MEM;
wire [AXI_DATA_WIDTH-1:0] read_data; 
wire [7:0] burst_length;        //: in integer range 1 to 256; -- number of beats in a burst
wire [6:0] burst_size;          //: in integer range 1 to 128;  -- number of byte lanes in each beat
wire increment_burst;   
wire clear_data_fifos;  
wire write_fifo_en;     
wire write_fifo_empty;  
wire write_fifo_full;   
wire read_fifo_en = CAPTURE;      
wire read_fifo_empty;   
wire read_fifo_full;    

    wire [16:0] DBG;
    wire WREN;
    wire [63:0] TO_MEM;
    wire [31:0] ADDR;
    logic [63:0] FROM_MEM;
    wire TCK;
    wire TCK2;
    wire RESET;
    wire RUNTEST;
    wire CAPTURE;
    wire CAPTURE2;
    wire UPDATE;
    wire UPDATE2;
   logic [63:0] sharedmem_dout;
   logic [7:0]  sharedmem_en;

    always @*
        begin      
        sharedmem_en = 8'h0;
        casez(ADDR[23:20])
            4'h8: begin sharedmem_en = 8'hff; FROM_MEM = sharedmem_dout; end
            default: FROM_MEM = 64'hDEADBEEF;
            endcase
        end

jtag_dummy jtag1(.*);

   genvar r;

   wire we_d, ce_d, wrap_sel;
   wire [7:0] m_enb = (we_d ? wrap_be : 8'hFF);
   wire m_web = ce_d & wrap_sel & we_d;
   logic [31:0] wrap_address;
   logic [31:0] wrap_address_dly;
   logic [63:0] wrap_wdata;
   logic  [3:0] wrap_be;
   logic [63:0] wrap_rdata;

   generate for (r = 0; r < 8; r=r+1)
     RAMB16_S9_S9
     RAMB16_S9_S9_inst
       (
        .CLKA   ( TCK                      ),     // Port A Clock
        .DOA    ( sharedmem_dout[r*8 +: 8] ),     // Port A 1-bit Data Output
        .DOPA   (                          ),
        .ADDRA  ( ADDR[13:3]               ),     // Port A 14-bit Address Input
        .DIA    ( TO_MEM[r*8 +:8]          ),     // Port A 1-bit Data Input
        .DIPA   ( 1'b0                     ),
        .ENA    ( sharedmem_en[r]          ),     // Port A RAM Enable Input
        .SSRA   ( 1'b0                     ),     // Port A Synchronous Set/Reset Input
        .WEA    ( WREN                     ),     // Port A Write Enable Input
        .CLKB   ( clk_i                    ),     // Port B Clock
        .DOB    ( wrap_rdata[r*8 +: 8]     ),     // Port B 1-bit Data Output
        .DOPB   (                          ),
        .ADDRB  ( wrap_address[13:3]       ),     // Port B 14-bit Address Input
        .DIB    ( wrap_wdata[r*8 +: 8]     ),     // Port B 1-bit Data Input
        .DIPB   ( 1'b0                     ),
        .ENB    ( m_enb[r]                 ),     // Port B RAM Enable Input
        .SSRB   ( 1'b0                     ),     // Port B Synchronous Set/Reset Input
        .WEB    ( m_web                    )      // Port B Write Enable Input
        );
   endgenerate

assign clear_data_fifos = &DBG;
assign {increment_burst, go, burst_size, burst_length} = DBG;
    // Add UUT instance   
AXI_master #(.DATA_WIDTH(AXI_DATA_WIDTH)) UUT
    ( 
    .go(go),                 
    .error(error),           
    .RNW(RNW),               
    .busy(busy),             
    .done(done),             
    .address(address),       
    .write_data(write_data), 
    .read_data(read_data),   
    .burst_length(burst_length),     
    .burst_size(burst_size),         
    .increment_burst(increment_burst),
    .clear_data_fifos(clear_data_fifos), 
    .write_fifo_en(write_fifo_en),       
    .write_fifo_empty(write_fifo_empty), 
    .write_fifo_full(write_fifo_full),   
    .read_fifo_en(read_fifo_en),    
    .read_fifo_empty(read_fifo_empty),   
    .read_fifo_full(read_fifo_full),
    .M_AXI_aclk     ( clk                  ), 
    .M_AXI_aresetn  ( rst_top              ),     
    .M_AXI_awvalid  ( dbg_master.aw_valid  ),
    .M_AXI_awaddr   ( dbg_master.aw_addr   ),
    .M_AXI_awprot   ( dbg_master.aw_prot   ),
    .M_AXI_awregion ( dbg_master.aw_region ),
    .M_AXI_awlen    ( dbg_master.aw_len    ),
    .M_AXI_awsize   ( dbg_master.aw_size   ),
    .M_AXI_awburst  ( dbg_master.aw_burst  ),
    .M_AXI_awlock   ( dbg_master.aw_lock   ),
    .M_AXI_awcache  ( dbg_master.aw_cache  ),
    .M_AXI_awqos    ( dbg_master.aw_qos    ),
    .M_AXI_awid     ( dbg_master.aw_id     ),
//    .M_AXI_awuser   ( dbg_master.aw_user   ),
    .M_AXI_awready  ( dbg_master.aw_ready  ),

    .M_AXI_arvalid  ( dbg_master.ar_valid  ),
    .M_AXI_araddr   ( dbg_master.ar_addr   ),
    .M_AXI_arprot   ( dbg_master.ar_prot   ),
    .M_AXI_arregion ( dbg_master.ar_region ),
    .M_AXI_arlen    ( dbg_master.ar_len    ),
    .M_AXI_arsize   ( dbg_master.ar_size   ),
    .M_AXI_arburst  ( dbg_master.ar_burst  ),
    .M_AXI_arlock   ( dbg_master.ar_lock   ),
    .M_AXI_arcache  ( dbg_master.ar_cache  ),
    .M_AXI_arqos    ( dbg_master.ar_qos    ),
    .M_AXI_arid     ( dbg_master.ar_id     ),
//    .M_AXI_aruser   ( dbg_master.ar_user   ),
    .M_AXI_arready  ( dbg_master.ar_ready  ),

    .M_AXI_wvalid   ( dbg_master.w_valid   ),
    .M_AXI_wdata    ( dbg_master.w_data    ),
    .M_AXI_wstrb    ( dbg_master.w_strb    ),
//    .M_AXI_wuser    ( dbg_master.w_user    ),
    .M_AXI_wlast    ( dbg_master.w_last    ),
    .M_AXI_wready   ( dbg_master.w_ready   ),

    .M_AXI_rvalid   ( dbg_master.r_valid   ),
    .M_AXI_rdata    ( dbg_master.r_data    ),
    .M_AXI_rresp    ( dbg_master.r_resp    ),
    .M_AXI_rlast    ( dbg_master.r_last    ),
    .M_AXI_rid      ( dbg_master.r_id      ),
//    .M_AXI_ruser    ( dbg_master.r_user    ),
    .M_AXI_rready   ( dbg_master.r_ready   ),

    .M_AXI_bvalid   ( dbg_master.b_valid   ),
    .M_AXI_bresp    ( dbg_master.b_resp    ),
    .M_AXI_bid      ( dbg_master.b_id      ),
//    .M_AXI_buser    ( dbg_master.b_user    ),
    .M_AXI_bready   ( dbg_master.b_ready   )
    );
    
assign {dbg_master.aw_user,dbg_master.ar_user,dbg_master.w_user,dbg_master.r_user,dbg_master.b_user} = 'b0;

   axi_ram_wrap #(
    .ID_WIDTH(AXI_ID_SLAVE_WIDTH),                 // id width
    .ADDR_WIDTH(AXI_ADDR_WIDTH),             // address width
    .DATA_WIDTH(AXI_DATA_WIDTH),             // width of data
    .USER_WIDTH(AXI_USER_WIDTH)              // width of user field, must > 0, let synthesizer trim it if not in use
    ) axi_mem1(.incoming_if(dbg_master));
 
display_top display(.clk    (clk),
                 .rst       (rst),
                 .bcd_digits(ADDR),
                 .CA        (CA),
                 .CB        (CB),
                 .CC        (CC),
                 .CD        (CD),
                 .CE        (CE),
                 .CF        (CF),
                 .CG        (CG),
                 .DP        (DP),
                 .AN        (AN));

endmodule
