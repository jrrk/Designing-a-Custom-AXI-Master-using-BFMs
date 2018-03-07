// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`default_nettype none

module dbg_wrap
#(
    parameter NB_CORES             = 1,
    parameter AXI_ADDR_WIDTH       = 32,
    parameter AXI_DATA_WIDTH       = 64,
    parameter AXI_ID_MASTER_WIDTH  = 4,
    parameter AXI_ID_SLAVE_WIDTH   = 4,
    parameter AXI_USER_WIDTH       = 0,
    parameter JTAG_CHAIN_START     = 1
 )
    (
    // Clock and Reset
    input logic         clk,
    input logic         rst_n,
    input  logic        testmode_i,
    output logic        aresetn,

    AXI_BUS.Master      dbg_master,
    // CPU signals
    output logic [15:0] cpu_addr_o, 
    input  logic [AXI_DATA_WIDTH-1:0] cpu_rdata_i, 
    output logic [AXI_DATA_WIDTH-1:0] cpu_wdata_o,
    input  logic        cpu_halted_i,
    output logic        cpu_halt_o,
    output logic        cpu_req_o,
    output logic        cpu_we_o,
    input  logic        cpu_gnt_i,
    output logic        cpu_resume_o,
    input  logic        cpu_rvalid_i,
    output logic        cpu_fetch_o,
  // JTAG signals
    input  logic        tck_i,
    input  logic        trstn_i,
    input  logic        tms_i,
    input  logic        tdi_i,
    output logic        tdo_o,
    output logic [63:0] boot_rdata,
    input  logic [15:0] boot_addr,
    input logic  [63:0] boot_wdata,
    input logic         boot_en,
    input logic  [7:0]  boot_we,
    output logic [31:0] address    
  );


wire combined_rstn = rst_n && trstn_i;
logic cpu_nofetch;   
assign cpu_fetch_o = !cpu_nofetch;
   
logic go;   
wire error;
logic RNW;  
wire busy; 
wire done; 
logic [7:0] burst_length;        //: in integer range 1 to 256; -- number of beats in a burst
logic [6:0] burst_size;          //: in integer range 1 to 128;  -- number of byte lanes in each beat
logic increment_burst;   
logic [31:0] wrap_address, next_wrap_address, shared_wrap_address;
logic [63:0] wrap_wdata, o_data;
logic [63:0] wrap_rdata;
logic [8:0] capture_address;
logic [63:0] capture_rdata;
logic [2:0] current_state;
logic [2:0] current_state_rac;
logic [2:0] current_state_resp;
logic [2:0] current_state_wrdata;
logic [9:0] start_out;
logic read_data_valid, write_data_valid, pc_asserted, cpu_capture;

wire [96 : 0] pc_status;

    wire [5:0] DBG;
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
   logic [63:0]  sharedmem_dout, bootmem_dout;
   logic [7:0]   sharedmem_en;
   logic [511:0] capmem_dout, capmem_shift, capture_wdata;
   logic [7:0]   bootmem_en;
   logic         burst_en, cpu_en, wrap_rst, capture_rst;
   logic [10:0]  unused1;
   logic         capmem_en;
   
   wire   capture_busy = ((go & busy) || cpu_capture) & !(&capture_address);
   logic  wrap_en;
   
   always @(posedge TCK or negedge combined_rstn)
     if (!combined_rstn)
       begin
          {capture_rst, wrap_rst, aresetn, go, increment_burst, RNW, burst_size, burst_length, address} <= 'b0;
          {cpu_capture, cpu_nofetch, cpu_resume_o, cpu_we_o, cpu_req_o, cpu_halt_o, cpu_addr_o, cpu_wdata_o} <= 'b0;
       end
     else
       begin
        if (burst_en)
          begin
             {unused1[10:0],capture_rst, wrap_rst, aresetn, go, increment_burst, RNW, burst_size, burst_length, address} <= TO_MEM;
          end
        if (cpu_en)
          begin
             if (ADDR[19])
               {cpu_capture, cpu_nofetch, cpu_resume_o, cpu_we_o, cpu_req_o, cpu_halt_o, cpu_addr_o} <= TO_MEM;
             else
               cpu_wdata_o <= TO_MEM;
          end
       end
   
    always @*
      begin
         wrap_en = write_data_valid || read_data_valid;
         next_wrap_address = wrap_address + {wrap_en,3'b000};
         shared_wrap_address = write_data_valid ? next_wrap_address : wrap_address;
         cpu_en = 1'b0; burst_en = 1'b0; sharedmem_en = 8'h0; wrap_en = 1'b1; capmem_en = 1'b0; bootmem_en = 8'b0;
         capmem_shift = capmem_dout >> {ADDR[5:3],6'b0};
         casez(ADDR[23:20])
           4'hf: begin cpu_en = &ADDR[31:24];
              FROM_MEM = ADDR[19] ? {cpu_rvalid_i, cpu_halted_i, cpu_gnt_i, cpu_capture, cpu_nofetch, cpu_resume_o, cpu_we_o, cpu_req_o, cpu_halt_o, cpu_addr_o} : cpu_rdata_i; end
           4'h9: begin capmem_en = 1'b1; FROM_MEM = capmem_shift[63:0]; end
           4'h8: begin sharedmem_en = 8'hff; wrap_en = 1'b0; FROM_MEM = sharedmem_dout; end
           4'h7: begin burst_en = 1'b1; FROM_MEM = {11'b0, capture_rst, wrap_rst, aresetn, go,
                                                    increment_burst, RNW, burst_size, burst_length, address}; end
           4'h6: begin FROM_MEM = {4'b0, current_state_wrdata, capture_busy, pc_asserted,
                                   current_state_resp, current_state_rac, start_out, aresetn,
                                   error, busy, done, current_state, wrap_address}; end
           4'h5: begin FROM_MEM = {31'b0, pc_status[96:64]}; end
           4'h4: begin FROM_MEM = pc_status[63:0]; end
           4'h3: begin FROM_MEM = {55'b0, capture_address}; end
           4'h2: begin bootmem_en = ADDR[31:24]; FROM_MEM = bootmem_dout; end
           default: FROM_MEM = 64'hDEADBEEF;
         endcase
      end

jtag_dummy #(.JTAG_CHAIN_START(JTAG_CHAIN_START)) jtag1(.*);

   genvar r;

   generate for (r = 0; r < 32; r=r+1)
     RAMB16_S2_S2
     RAMB16_S2_S2_inst
       (
        .CLKA   ( TCK                      ),     // Port A Clock
        .DOA    ( bootmem_dout[r*2 +: 2]   ),     // Port A 1-bit Data Output
        .ADDRA  ( ADDR[15:3]               ),     // Port A 14-bit Address Input
        .DIA    ( TO_MEM[r*2 +:2]          ),     // Port A 1-bit Data Input
        .ENA    ( bootmem_en[r/4]          ),     // Port A RAM Enable Input
        .SSRA   ( 1'b0                     ),     // Port A Synchronous Set/Reset Input
        .WEA    ( WREN                     ),     // Port A Write Enable Input
        .CLKB   ( clk                      ),     // Port B Clock
        .DOB    ( boot_rdata[r*2 +: 2]     ),     // Port B 1-bit Data Output
        .ADDRB  ( boot_addr[15:3]          ),     // Port B 14-bit Address Input
        .DIB    ( boot_wdata[r*2 +: 2]     ),     // Port B 1-bit Data Input
        .ENB    ( boot_en                  ),     // Port B RAM Enable Input
        .SSRB   ( 1'b0                     ),     // Port B Synchronous Set/Reset Input
        .WEB    ( boot_we[r/4]             )      // Port B Write Enable Input
        );
   endgenerate

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
        .CLKB   ( clk                      ),     // Port B Clock
        .DOB    ( wrap_rdata[r*8 +: 8]     ),     // Port B 1-bit Data Output
        .DOPB   (                          ),
        .ADDRB  ( shared_wrap_address[13:3] ),     // Port B 14-bit Address Input
        .DIB    ( wrap_wdata[r*8 +: 8]     ),     // Port B 1-bit Data Input
        .DIPB   ( 1'b0                     ),
        .ENB    ( wrap_en                  ),     // Port B RAM Enable Input
        .SSRB   ( 1'b0                     ),     // Port B Synchronous Set/Reset Input
        .WEB    ( read_data_valid          )      // Port B Write Enable Input
        );
   endgenerate

   generate for (r = 0; r < 16; r=r+1)
     RAMB16_S36_S36
     RAMB16_S36_S36_inst
       (
        .CLKA   ( TCK                      ),     // Port A Clock
        .DOA    ( capmem_dout[r*32 +: 32]  ),     // Port A 1-bit Data Output
        .DOPA   (                          ),
        .ADDRA  ( ADDR[14:6]               ),     // Port A 14-bit Address Input
        .DIA    ( 32'b0                    ),     // Port A 1-bit Data Input
        .DIPA   ( 4'b0                     ),
        .ENA    ( capmem_en                ),     // Port A RAM Enable Input
        .SSRA   ( 1'b0                     ),     // Port A Synchronous Set/Reset Input
        .WEA    ( 1'b0                     ),     // Port A Write Enable Input
        .CLKB   ( clk                      ),     // Port B Clock
        .DOB    (                          ),     // Port B 1-bit Data Output
        .DOPB   (                          ),
        .ADDRB  ( capture_address          ),     // Port B 14-bit Address Input
        .DIB    ( capture_wdata[r*32 +: 32]),     // Port B 1-bit Data Input
        .DIPB   ( 4'b0                     ),
        .ENB    ( capture_busy             ),     // Port B RAM Enable Input
        .SSRB   ( 1'b0                     ),     // Port B Synchronous Set/Reset Input
        .WEB    ( capture_busy             )      // Port B Write Enable Input
        );
   endgenerate

   wire [AXI_DATA_WIDTH-1:0] dbg_r_data32 = dbg_master.r_data;
   wire [AXI_DATA_WIDTH-1:0] dbg_w_data32 = dbg_master.w_data;

always @(posedge clk)
    begin
       if (wrap_rst)
         wrap_address <= address[13:0];
       else
         wrap_address <= next_wrap_address;
       if (capture_rst)
         capture_address <= 'b0;
       else if (capture_busy)
         capture_address <= capture_address + 9'b1;
       capture_wdata <= {
 wrap_rdata,
 wrap_en,
 wrap_address[13:0],
 read_data_valid,
 write_data_valid,
 boot_rdata,
 boot_addr,
 boot_wdata,
 boot_en,
 boot_we,
 capture_address[8:0],
 current_state_resp,                         
 current_state,                         
 current_state_wrdata,                         
 current_state_rac,
 start_out,                      
 done,
 busy,
 error,
 dbg_master.aw_valid  ,
 dbg_master.aw_prot   ,
 dbg_master.aw_region ,
 dbg_master.aw_len    ,
 dbg_master.aw_size   ,
 dbg_master.aw_burst  ,
 dbg_master.aw_lock   ,
 dbg_master.aw_cache  ,
 dbg_master.aw_qos    ,
 dbg_master.aw_id     ,
 dbg_master.aw_ready  ,
 dbg_master.ar_valid  ,
 dbg_master.ar_prot   ,
 dbg_master.ar_region ,
 dbg_master.ar_len    ,
 dbg_master.ar_size   ,
 dbg_master.ar_burst  ,
 dbg_master.ar_lock   ,
 dbg_master.ar_cache  ,
 dbg_master.ar_qos    ,
 dbg_master.ar_id     ,
 dbg_master.ar_ready  ,
 dbg_master.w_valid   ,
 dbg_master.w_strb    ,
 dbg_master.w_last    ,
 dbg_master.w_ready   ,
 dbg_master.r_valid   ,
 dbg_master.r_resp    ,
 dbg_master.r_last    ,
 dbg_master.r_id      ,
 dbg_master.r_ready   ,
 dbg_master.b_valid   ,
 dbg_master.b_resp    ,
 dbg_master.b_id      ,
 dbg_master.b_ready   ,
 dbg_master.w_data    ,
 dbg_master.r_data    ,
 dbg_master.aw_addr   ,
 dbg_master.ar_addr   
  };
       
    end
    
    // Add UUT instance   
AXI_master #(.data_width(AXI_DATA_WIDTH)) UUT
    ( 
    .go(go),                 
    .error(error),           
    .RNW(RNW),               
    .busy(busy),             
    .done(done),             
    .address(address),       
    .write_data(wrap_rdata), 
    .read_data(wrap_wdata),   
    .burst_length(burst_length),     
    .burst_size(burst_size),         
    .increment_burst(increment_burst),
    .read_data_valid(read_data_valid),
    .write_data_valid(write_data_valid),
    .current_state_out(current_state),
    .current_state_rac(current_state_rac),
    .current_state_resp(current_state_resp),
    .current_state_wrdata(current_state_wrdata),
    .start_out(start_out),
    .m_axi_aclk     ( clk                  ), 
    .m_axi_aresetn  ( aresetn              ),     
    .m_axi_awvalid  ( dbg_master.aw_valid  ),
    .m_axi_awaddr   ( dbg_master.aw_addr   ),
    .m_axi_awprot   ( dbg_master.aw_prot   ),
    .m_axi_awregion ( dbg_master.aw_region ),
    .m_axi_awlen    ( dbg_master.aw_len    ),
    .m_axi_awsize   ( dbg_master.aw_size   ),
    .m_axi_awburst  ( dbg_master.aw_burst  ),
    .m_axi_awlock   ( dbg_master.aw_lock   ),
    .m_axi_awcache  ( dbg_master.aw_cache  ),
    .m_axi_awqos    ( dbg_master.aw_qos    ),
    .m_axi_awid     ( dbg_master.aw_id     ),
//    .m_axi_awuser   ( dbg_master.aw_user   ),
    .m_axi_awready  ( dbg_master.aw_ready  ),

    .m_axi_arvalid  ( dbg_master.ar_valid  ),
    .m_axi_araddr   ( dbg_master.ar_addr   ),
    .m_axi_arprot   ( dbg_master.ar_prot   ),
    .m_axi_arregion ( dbg_master.ar_region ),
    .m_axi_arlen    ( dbg_master.ar_len    ),
    .m_axi_arsize   ( dbg_master.ar_size   ),
    .m_axi_arburst  ( dbg_master.ar_burst  ),
    .m_axi_arlock   ( dbg_master.ar_lock   ),
    .m_axi_arcache  ( dbg_master.ar_cache  ),
    .m_axi_arqos    ( dbg_master.ar_qos    ),
    .m_axi_arid     ( dbg_master.ar_id     ),
//    .m_axi_aruser   ( dbg_master.ar_user   ),
    .m_axi_arready  ( dbg_master.ar_ready  ),

    .m_axi_wvalid   ( dbg_master.w_valid   ),
    .m_axi_wid      (                      ),
    .m_axi_wdata    ( dbg_master.w_data    ),
    .m_axi_wstrb    ( dbg_master.w_strb    ),
//    .m_axi_wuser    ( dbg_master.w_user    ),
    .m_axi_wlast    ( dbg_master.w_last    ),
    .m_axi_wready   ( dbg_master.w_ready   ),

    .m_axi_rvalid   ( dbg_master.r_valid   ),
    .m_axi_rdata    ( dbg_master.r_data    ),
    .m_axi_rresp    ( dbg_master.r_resp    ),
    .m_axi_rlast    ( dbg_master.r_last    ),
    .m_axi_rid      ( dbg_master.r_id      ),
//    .m_axi_ruser    ( dbg_master.r_user    ),
    .m_axi_rready   ( dbg_master.r_ready   ),

    .m_axi_bvalid   ( dbg_master.b_valid   ),
    .m_axi_bresp    ( dbg_master.b_resp    ),
    .m_axi_bid      ( dbg_master.b_id      ),
//    .m_axi_buser    ( dbg_master.b_user    ),
    .m_axi_bready   ( dbg_master.b_ready   )
    );
    
assign {dbg_master.aw_user,dbg_master.ar_user,dbg_master.w_user} = 'b0;
    
`ifdef PROTO_WRAPPER
   axi_proto_wrap #(
    .ID_WIDTH(AXI_ID_SLAVE_WIDTH),           // id width
    .ADDR_WIDTH(AXI_ADDR_WIDTH),             // address width
    .DATA_WIDTH(AXI_DATA_WIDTH),             // width of data
    .USER_WIDTH(AXI_USER_WIDTH)              // width of user field, must > 0, let synthesizer trim it if not in use
    ) axi_proto1(
        .clk(clk),
        .aresetn(aresetn),
        .pc_status(pc_status),              // output wire [96 : 0] pc_status
        .pc_asserted(pc_asserted),          // output wire pc_asserted
        .proto_if(dbg_master));
`endif

`ifdef PROTO_CHECKER
axi_protocol_checker_0 pc1 (
  .pc_status(pc_status),              // output wire [96 : 0] pc_status
  .pc_asserted(pc_asserted),          // output wire pc_asserted
  .system_resetn(aresetn),            // input wire system_resetn
  .aclk(clk),                         // input wire aclk
  .aresetn(aresetn),                  // input wire aresetn
  .pc_axi_awid(dbg_master.aw_id),
  .pc_axi_awaddr(dbg_master.aw_addr),
  .pc_axi_awlen(dbg_master.aw_len),
  .pc_axi_awsize(dbg_master.aw_size),
  .pc_axi_awburst(dbg_master.aw_burst),
  .pc_axi_awlock(dbg_master.aw_lock),
  .pc_axi_awcache(dbg_master.aw_cache),
  .pc_axi_awprot(dbg_master.aw_prot),
  .pc_axi_awregion(dbg_master.aw_region),
  .pc_axi_awqos(dbg_master.aw_qos),
  .pc_axi_awuser(dbg_master.aw_user),
  .pc_axi_awvalid(dbg_master.aw_valid),
  .pc_axi_awready(dbg_master.aw_ready),
  .pc_axi_wdata(dbg_master.w_data),
  .pc_axi_wstrb(dbg_master.w_strb),
  .pc_axi_wlast(dbg_master.w_last),
  .pc_axi_wuser(dbg_master.w_user),
  .pc_axi_wvalid(dbg_master.w_valid),
  .pc_axi_wready(dbg_master.w_ready),
  .pc_axi_bid(dbg_master.b_id),
  .pc_axi_bresp(dbg_master.b_resp),
  .pc_axi_buser(dbg_master.b_user),
  .pc_axi_bvalid(dbg_master.b_valid),
  .pc_axi_bready(dbg_master.b_ready),
  .pc_axi_arid(dbg_master.ar_id),
  .pc_axi_araddr(dbg_master.ar_addr),
  .pc_axi_arlen(dbg_master.ar_len),
  .pc_axi_arsize(dbg_master.ar_size),
  .pc_axi_arburst(dbg_master.ar_burst),
  .pc_axi_arlock(dbg_master.ar_lock),
  .pc_axi_arcache(dbg_master.ar_cache),
  .pc_axi_arprot(dbg_master.ar_prot),
  .pc_axi_arregion(dbg_master.ar_region),
  .pc_axi_arqos(dbg_master.ar_qos),
  .pc_axi_aruser(dbg_master.ar_user),
  .pc_axi_arvalid(dbg_master.ar_valid),
  .pc_axi_arready(dbg_master.ar_ready),
  .pc_axi_rid(dbg_master.r_id),
  .pc_axi_rdata(dbg_master.r_data),
  .pc_axi_rresp(dbg_master.r_resp),
  .pc_axi_rlast(dbg_master.r_last),
  .pc_axi_ruser(dbg_master.r_user),
  .pc_axi_rvalid(dbg_master.r_valid),
  .pc_axi_rready(dbg_master.r_ready)
);
`endif

endmodule
