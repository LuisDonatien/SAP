// Copyright 2025 CEI UPM
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Luis Waucquez (luis.waucquez.jimenez@upm.es)

module eros_top_wrapper_axi
  import obi_pkg::*;
  import reg_pkg::*;
  import core_v_mini_mcu_pkg::*;
  import eros_pkg::*;
#(
    parameter NHARTS  = 3,
    parameter N_BANKS = 2,
    parameter int C_S00_AXI_DATA_WIDTH = 32,
    parameter int C_S00_AXI_ADDR_WIDTH = 32,
    parameter int C_S01_AXI_DATA_WIDTH = 32,
    parameter int C_S01_AXI_ADDR_WIDTH = 32
) (
    // Clock and Reset
    input logic clk_i,
    input logic rst_ni,

    // Top level clock gating unit enable
    input logic en_i,

    //Bus External Slave
    output obi_req_t  ext_slave_req_o,
    input  obi_resp_t ext_slave_resp_i,

    // ----------------------------------------------
    // Ports of Axi Slave Bus Interface S00_AXI -> OBI -> REG
    // ----------------------------------------------
    // Read address channel signals
    input  logic [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
    input  logic                         s00_axi_arvalid,
    output logic                         s00_axi_arready,
    input  logic [2:0]                   s00_axi_arprot, // not used

    // Read data channel signals
    output logic [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_rdata,
    output logic [1:0]                   s00_axi_rresp,
    output logic                         s00_axi_rvalid,
    input  logic                         s00_axi_rready,

    // Write address channel signals
    input  logic [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
    input  logic                         s00_axi_awvalid,
    output logic                         s00_axi_awready,
    input  logic [2:0]                   s00_axi_awprot, // not used

    // Write data channel signals
    input  logic [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_wdata,
    input  logic                         s00_axi_wvalid,
    output logic                         s00_axi_wready,
    input  logic [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb, // not used

    // Write response channel signals
    output logic [1:0]                   s00_axi_bresp,
    output logic                         s00_axi_bvalid,
    input  logic                         s00_axi_bready

    // ----------------------------------------------
    // ----------------------------------------------
    // ----------------------------------------------

    // ----------------------------------------------
    // Ports of Axi Slave Bus Interface S00_AXI -> OBI
    // ----------------------------------------------
    // Read address channel signals
    input  logic [C_S01_AXI_ADDR_WIDTH-1:0] s01_axi_araddr,
    input  logic                         s01_axi_arvalid,
    output logic                         s01_axi_arready,
    input  logic [2:0]                   s01_axi_arprot, // not used

    // Read data channel signals
    output logic [C_S01_AXI_DATA_WIDTH-1:0] s01_axi_rdata,
    output logic [1:0]                   s01_axi_rresp,
    output logic                         s01_axi_rvalid,
    input  logic                         s01_axi_rready,

    // Write address channel signals
    input  logic [C_S01_AXI_ADDR_WIDTH-1:0] s01_axi_awaddr,
    input  logic                         s01_axi_awvalid,
    output logic                         s01_axi_awready,
    input  logic [2:0]                   s01_axi_awprot, // not used

    // Write data channel signals
    input  logic [C_S01_AXI_DATA_WIDTH-1:0] s01_axi_wdata,
    input  logic                         s01_axi_wvalid,
    output logic                         s01_axi_wready,
    input  logic [(C_S01_AXI_DATA_WIDTH/8)-1:0] s01_axi_wstrb, // not used

    // Write response channel signals
    output logic [1:0]                   s01_axi_bresp,
    output logic                         s01_axi_bvalid,
    input  logic                         s01_axi_bready

    // Debug Interface
    input  logic              debug_req_i,
    output logic [NHARTS-1:0] sleep_o,

    // power manager signals that goes to the ASIC macros
    input  logic [N_BANKS-1:0] pwrgate_ni,
    output logic [N_BANKS-1:0] pwrgate_ack_no,
    input  logic [N_BANKS-1:0] set_retentive_ni,

    // Interrupt Interface
    output logic interrupt_o
);

  // Slave AXI - Slave OBI
  obi_req_t     axi_obi_master_req;
  obi_resp_t    axi_obi_master_resp;

  // Slave AXI-LITE - Slave REG
  obi_req_t     axi_obi_reg_master_req;
  obi_resp_t    axi_obi_reg_master_resp;

  reg_pkg::reg_req_t axi_reg_master_req;
  reg_pkg::reg_rsp_t axi_reg_master_rsp;



  logic clk_cg;

  eros_clock_gate eros_clock_gate_i (
      .clk_i    (clk_i),
      .test_en_i(1'b0),
      .en_i     (en_i),
      .clk_o    (clk_cg)
  );



  eros_top eros_top_i (
      .clk_i(clk_cg),
      .rst_ni,
      .ext_master_req_i(axi_obi_master_req),
      .ext_master_resp_o(axi_obi_master_resp),
      .ext_slave_req_o,
      .ext_slave_resp_i,
      .csr_reg_req_i(axi_reg_master_req),
      .csr_reg_resp_o(axi_reg_master_rsp),
      .debug_req_i,
      .pwrgate_ni,
      .pwrgate_ack_no,
      .set_retentive_ni,
      .sleep_o,
      .interrupt_o
  );

  //AXI-> OBI MASTER
  axi2obi #(
      .C_S00_AXI_DATA_WIDTH,
      .C_S00_AXI_ADDR_WIDTH
  )axi2_obi_i(

    .clk_i,
    .rst_ni,

    .gnt_i(axi_obi_master_resp.gnt),
    .rvalid_i(axi_obi_master_resp.rvalid),
    .rdata_i(axi_obi_master_resp.rdata),
    .we_o(axi_obi_master_req.we),
    .be_o(axi_obi_master_req.be),
    .addr_o(axi_obi_master_req.addr),
    .wdata_o(axi_obi_master_req.wdata),
    .req_o(axi_obi_master_req.req),

    .s00_axi_araddr,
    .s00_axi_arvalid,
    .s00_axi_arready,
    .s00_axi_arprot,

    .s00_axi_rdata,
    .s00_axi_rresp,
    .s00_axi_rvalid,
    .s00_axi_rready,

    .s00_axi_awaddr,
    .s00_axi_awvalid,
    .s00_axi_awready,
    .s00_axi_awprot,

    .s00_axi_wdata,
    .s00_axi_wvalid,
    .s00_axi_wready,
    .s00_axi_wstrb,

    .s00_axi_bresp,
    .s00_axi_bvalid,
    .s00_axi_bready
);
  //AXI -> OBI_REG MASTER
  axi2obi #(
      .C_S01_AXI_DATA_WIDTH,
      .C_S01_AXI_ADDR_WIDTH
  )axi2_obi_i(

    .clk_i,
    .rst_ni,

    .gnt_i(axi_obi_reg_master_resp.gnt),
    .rvalid_i(axi_obi_reg_master_resp.rvalid),
    .rdata_i(axi_obi_reg_master_resp.rdata),
    .we_o(axi_obi_reg_master_req.we),
    .be_o(axi_obi_reg_master_req.be),
    .addr_o(axi_obi_reg_master_req.addr),
    .wdata_o(axi_obi_reg_master_req.wdata),
    .req_o(axi_obi_reg_master_req.req),

    .s01_axi_araddr,
    .s01_axi_arvalid,
    .s01_axi_arready,
    .s01_axi_arprot, // not used

    .s01_axi_rdata,
    .s01_axi_rresp,
    .s01_axi_rvalid,
    .s01_axi_rready,

    .s01_axi_awaddr,
    .s01_axi_awvalid,
    .s01_axi_awready,
    .s01_axi_awprot, // not used

    .s01_axi_wdata,
    .s01_axi_wvalid,
    .s01_axi_wready,
    .s01_axi_wstrb, // not used

    .s01_axi_bresp,
    .s01_axi_bvalid,
    .s01_axi_bready
);
  //AXI OBI_REG MASTER -> REG MASTER
  periph_to_reg #(
      .req_t(reg_pkg::reg_req_t),
      .rsp_t(reg_pkg::reg_rsp_t),
      .IW(1)
  ) cpu_periph_to_reg_i (
      .clk_i,
      .rst_ni,
      .req_i(axi_obi_reg_master_req.req),
      .add_i(axi_obi_reg_master_req.addr),
      .wen_i(~axi_obi_reg_master_req.we),
      .wdata_i(axi_obi_reg_master_req.wdata),
      .be_i(axi_obi_reg_master_req.be),
      .id_i('0),
      .gnt_o(axi_obi_reg_master_resp.gnt),
      .r_rdata_o(axi_obi_reg_master_resp.rdata),
      .r_opc_o(),
      .r_id_o(),
      .r_valid_o(axi_obi_reg_master_resp.rvalid),
      .reg_req_o(axi_reg_master_req),
      .reg_rsp_i(axi_reg_master_rsp)
  );

endmodule
