// Copyright 2025 CEI UPM
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Luis Waucquez (luis.waucquez.jimenez@upm.es)

`include "common_cells/assertions.svh"

module cpu_private_reg #(
    parameter type reg_req_t = logic,
    parameter type reg_rsp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    // Bus Interface
    input  reg_req_t reg_req_i,
    output reg_rsp_t reg_rsp_o,

    //Output
    input logic [2:0] Core_id_i,
    output logic Hart_intc_ack_o

);

  import cpu_private_reg_pkg::*;

  cpu_private_reg2hw_t reg2hw;
  cpu_private_hw2reg_t hw2reg;

  cpu_private_reg_top #(
      .reg_req_t(reg_req_t),
      .reg_rsp_t(reg_rsp_t)
  ) cpu_private_reg_top_i (
      .clk_i,
      .rst_ni,
      .reg_req_i,
      .reg_rsp_o,
      .reg2hw,
      .hw2reg,
      .devmode_i(1'b1)
  );

  //Reg2Hw read
  assign Hart_intc_ack_o   = reg2hw.hart_intc_ack.q;

  //Hw2Reg always write
  assign hw2reg.core_id.d  = Core_id_i;
  assign hw2reg.core_id.de = 1'b1;


endmodule : cpu_private_reg
