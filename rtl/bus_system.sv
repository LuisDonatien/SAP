// Copyright 2017 Embecosm Limited <www.embecosm.com>
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// System bus for core-v-mini-mcu
// Contributor: Jeremy Bennett <jeremy.bennett@embecosm.com>
//              Robert Balas <balasr@student.ethz.ch>
//              Davide Schiavone <davide@openhwgroup.org>
//              Simone Machetti <simone.machetti@epfl.ch>
//              Michele Caon <michele.caon@epfl.ch>

module bus_system
  import obi_pkg::*;
  import reg_pkg::*;
  import addr_map_rule_pkg::*;
#(
  parameter NHARTS = 3,
  parameter N_BANKS = 2
) (
    input logic clk_i,
    input logic rst_ni,

    // Internal master ports
    input  obi_req_t  [NHARTS-1 : 0] core_instr_req_i,
    output obi_resp_t [NHARTS-1 : 0] core_instr_resp_o,

    input  obi_req_t  [NHARTS-1 : 0] core_data_req_i,
    output obi_resp_t [NHARTS-1 : 0] core_data_resp_o,

    // Internal slave ports
    output obi_req_t  peripheral_slave_req_o,
    input  obi_resp_t peripheral_slave_resp_i,

    //External master
    input  obi_req_t    ext_master_req_i,
    output obi_resp_t   ext_master_resp_o,

    //CSR 
    input  reg_req_t    ext_csr_reg_req_i,
    output reg_rsp_t    ext_csr_reg_resp_o,    

    //External slave
    output obi_req_t    ext_slave_req_o,
    input  obi_resp_t   ext_slave_resp_i,

    //Ram memory
    output obi_req_t   [N_BANKS-1:0]ram_req_o,
    input  obi_resp_t  [N_BANKS-1:0]ram_resp_i,

    // Control Status Register Output 
    output reg_req_t   wrapper_csr_req_o,
    input  reg_rsp_t   wrapper_csr_rsp_i

);

  import cei_mochila_pkg::*;

    obi_req_t  core_instr_req_s; 
    obi_req_t  core_data_req_s; 


obi_sngreg obi_sngreg_0i (
    .clk_i,
    .rst_ni,
    .clear_pipeline('0),

    .core_instr_req_i(core_instr_req_i[0]),
    .core_instr_req_o(core_instr_req_s),
    .core_instr_resp_gnt_i(int_master_resp[cei_mochila_pkg::CORE0_INSTR_IDX].gnt),
    .core_instr_resp_gnt_o(core_instr_resp_o[0].gnt)
);

obi_sngreg obi_sngreg_1i (
    .clk_i,
    .rst_ni,
    .clear_pipeline('0),

    .core_instr_req_i(core_data_req_i[0]),
    .core_instr_req_o(core_data_req_s),
    .core_instr_resp_gnt_i(int_master_resp[cei_mochila_pkg::CORE0_DATA_IDX].gnt),
    .core_instr_resp_gnt_o(core_data_resp_o[0].gnt)
);


  // Safe CPU reg port
  reg_pkg::reg_req_t  int_safe_cpu_wrapper_reg_req;
  reg_pkg::reg_rsp_t  int_safe_cpu_wrapper_reg_rsp;   

  reg_pkg::reg_req_t  [1:0] int_req;
  reg_pkg::reg_rsp_t  [1:0] int_rsp;  

  obi_req_t  int_wrapper_csr_req;
  obi_resp_t int_wrapper_csr_resp;   

  // Internal master ports
  obi_req_t [cei_mochila_pkg::SYSTEM_XBAR_NMASTER-1:0] int_master_req;
  obi_resp_t [cei_mochila_pkg::SYSTEM_XBAR_NMASTER-1:0] int_master_resp;

  // Internal slave ports
  obi_req_t [cei_mochila_pkg::SYSTEM_XBAR_NSLAVE-1:0] int_slave_req;
  obi_resp_t [cei_mochila_pkg::SYSTEM_XBAR_NSLAVE-1:0] int_slave_resp;

  // Internal master requests

  assign int_master_req[cei_mochila_pkg::CORE0_INSTR_IDX] = core_instr_req_s;
  assign int_master_req[cei_mochila_pkg::CORE0_DATA_IDX] = core_data_req_s;

//  assign int_master_req[cei_mochila_pkg::CORE0_INSTR_IDX] = core_instr_req_i[0];
//  assign int_master_req[cei_mochila_pkg::CORE0_DATA_IDX] = core_data_req_i[0];
  assign int_master_req[cei_mochila_pkg::CORE1_INSTR_IDX] = core_instr_req_i[1];
  assign int_master_req[cei_mochila_pkg::CORE1_DATA_IDX] = core_data_req_i[1];
  assign int_master_req[cei_mochila_pkg::CORE2_INSTR_IDX] = core_instr_req_i[2];
  assign int_master_req[cei_mochila_pkg::CORE2_DATA_IDX] = core_data_req_i[2];
  assign int_master_req[cei_mochila_pkg::EXTERNAL_MASTER_IDX] = ext_master_req_i; 

  // Internal master responses

  assign core_instr_resp_o[0].rvalid = int_master_resp[cei_mochila_pkg::CORE0_INSTR_IDX].rvalid;
  assign core_instr_resp_o[0].rdata = int_master_resp[cei_mochila_pkg::CORE0_INSTR_IDX].rdata;

  assign core_data_resp_o[0].rvalid = int_master_resp[cei_mochila_pkg::CORE0_DATA_IDX].rvalid;
  assign core_data_resp_o[0].rdata = int_master_resp[cei_mochila_pkg::CORE0_DATA_IDX].rdata;

//  assign core_instr_resp_o[0] = int_master_resp[cei_mochila_pkg::CORE0_INSTR_IDX];
//  assign core_data_resp_o[0] = int_master_resp[cei_mochila_pkg::CORE0_DATA_IDX];
  assign core_instr_resp_o[1] = int_master_resp[cei_mochila_pkg::CORE1_INSTR_IDX];
  assign core_data_resp_o[1] = int_master_resp[cei_mochila_pkg::CORE1_DATA_IDX];
  assign core_instr_resp_o[2] = int_master_resp[cei_mochila_pkg::CORE2_INSTR_IDX];
  assign core_data_resp_o[2] = int_master_resp[cei_mochila_pkg::CORE2_DATA_IDX];
  // External master responses
  assign ext_master_resp_o = int_master_resp[cei_mochila_pkg::EXTERNAL_MASTER_IDX];

  // Internal slave requests
  assign peripheral_slave_req_o = int_slave_req[cei_mochila_pkg::PERIPHERAL_IDX];
  assign ram_req_o[0]           = int_slave_req[cei_mochila_pkg::MEMORY_RAM0_IDX];
  assign ram_req_o[1]           = int_slave_req[cei_mochila_pkg::MEMORY_RAM1_IDX];
  assign int_wrapper_csr_req      = int_slave_req[cei_mochila_pkg::SAFE_CPU_REGISTER_IDX]; 

  // External slave requests
  assign ext_slave_req_o = int_slave_req[cei_mochila_pkg::EXTERNAL_PERIPHERAL_IDX];

  // Internal slave responses
  assign int_slave_resp[cei_mochila_pkg::PERIPHERAL_IDX] = peripheral_slave_resp_i;
  assign int_slave_resp[cei_mochila_pkg::MEMORY_RAM0_IDX] = ram_resp_i[0];
  assign int_slave_resp[cei_mochila_pkg::MEMORY_RAM1_IDX] = ram_resp_i[1];
  assign int_slave_resp[cei_mochila_pkg::SAFE_CPU_REGISTER_IDX] = int_wrapper_csr_resp;
  // External slave responses
  assign int_slave_resp[cei_mochila_pkg::EXTERNAL_PERIPHERAL_IDX] = ext_slave_resp_i;
  // Internal system crossbar
  // ------------------------
  xbar_system #(
      .XBAR_NMASTER(cei_mochila_pkg::SYSTEM_XBAR_NMASTER),
      .XBAR_NSLAVE (cei_mochila_pkg::SYSTEM_XBAR_NSLAVE)
  ) xbar_system_i (
      .clk_i(clk_i),
      .rst_ni(rst_ni),
      .addr_map_i(cei_mochila_pkg::XBAR_ADDR_RULES),
      .default_idx_i(cei_mochila_pkg::ERROR_IDX[LOG_SYSTEM_XBAR_NSLAVE-1:0]),
      .master_req_i(int_master_req),
      .master_resp_o(int_master_resp),
      .slave_req_o(int_slave_req),
      .slave_resp_i(int_slave_resp)
  );


//***OBI Slave[1] -> Safe CPU Wrapper Register***//
   periph_to_reg #(
       .req_t(reg_pkg::reg_req_t),
       .rsp_t(reg_pkg::reg_rsp_t),
       .IW(1)
   ) cpu_periph_to_reg_i (
       .clk_i,
       .rst_ni,
       .req_i(int_wrapper_csr_req.req),
       .add_i(int_wrapper_csr_req.addr),
       .wen_i(~int_wrapper_csr_req.we),
       .wdata_i(int_wrapper_csr_req.wdata),
       .be_i(int_wrapper_csr_req.be),
       .id_i('0),
       .gnt_o(int_wrapper_csr_resp.gnt),
       .r_rdata_o(int_wrapper_csr_resp.rdata),
       .r_opc_o(),
       .r_id_o(),
       .r_valid_o(int_wrapper_csr_resp.rvalid),
       .reg_req_o(int_safe_cpu_wrapper_reg_req),
       .reg_rsp_i(int_safe_cpu_wrapper_reg_rsp)
  );

  assign int_req[1] = int_safe_cpu_wrapper_reg_req;
  assign int_safe_cpu_wrapper_reg_rsp = int_rsp[1];
  assign int_req[0] = ext_csr_reg_req_i;
  assign ext_csr_reg_resp_o = int_rsp[0];


  reg_mux #(
    .NoPorts(2),
    .AW(32),
    .DW(32),
    .req_t(reg_pkg::reg_req_t),
    .rsp_t(reg_pkg::reg_rsp_t)
  ) reg_mux_i(
    .clk_i,
    .rst_ni,
    .in_req_i(int_req),
    .in_rsp_o(int_rsp),
    .out_req_o(wrapper_csr_req_o),
    .out_rsp_i(wrapper_csr_rsp_i)
  );
endmodule
