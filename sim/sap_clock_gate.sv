// Copyright 2025 CEI UPM
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Luis Waucquez (luis.waucquez.jimenez@upm.es)

module sap_clock_gate (
    input  logic clk_i,
    input  logic test_en_i,
    input  logic en_i,
    output logic clk_o
);

  // For synthesis replace with library clock-gating cells
  logic clk_en;

  always_latch begin
    if (clk_i == 1'b0) clk_en <= en_i | test_en_i;
  end

  assign clk_o = clk_i & clk_en;

endmodule
