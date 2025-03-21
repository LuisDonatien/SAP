// Copyright 2025 CEI UPM
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Luis Waucquez (luis.waucquez.jimenez@upm.es)

module lockstep_reg
  import obi_pkg::*;
  import reg_pkg::*;
#(
    parameter NHARTS  = 3,
    parameter NCYCLES = 2
) (
    input logic clk_i,
    input logic rst_ni,
    input logic enable_i,
    input logic [NHARTS-1:0] mask,

    input  obi_req_t [NHARTS-1:0] core_instr_req_i,
    output obi_req_t [NHARTS-1:0] core_instr_req_o,

    input  obi_resp_t [NHARTS-1:0] core_instr_resp_i,
    output obi_resp_t [NHARTS-1:0] core_instr_resp_o,

    input  obi_req_t [NHARTS-1:0] core_data_req_i,
    output obi_req_t [NHARTS-1:0] core_data_req_o,

    input  obi_resp_t [NHARTS-1:0] core_data_resp_i,
    output obi_resp_t [NHARTS-1:0] core_data_resp_o,

    input  logic [NHARTS-1:0][31:0] intr_i,
    output logic [NHARTS-1:0][31:0] intr_o,
    input  logic [NHARTS-1:0]       debug_i,
    output logic [NHARTS-1:0]       debug_o
);


  obi_req_t  [             NHARTS-1:0      ] mux_core_instr_req_i;
  obi_req_t  [             NHARTS-1:0      ] mux_core_instr_req_o;

  obi_resp_t [             NHARTS-1:0      ] mux_core_instr_resp_i;
  obi_resp_t [             NHARTS-1:0      ] mux_core_instr_resp_o;

  obi_req_t  [             NHARTS-1:0      ] mux_core_data_req_i;
  obi_req_t  [             NHARTS-1:0      ] mux_core_data_req_o;

  obi_resp_t [             NHARTS-1:0      ] mux_core_data_resp_i;
  obi_resp_t [             NHARTS-1:0      ] mux_core_data_resp_o;

  logic        [NHARTS-1:0]          [31:0]  mux_intr_i;
  logic        [NHARTS-1:0]          [31:0]  mux_intr_o;
  logic        [NHARTS-1:0]                  mux_debug_req_i;
  logic        [NHARTS-1:0]                  mux_debug_req_o;


  logic pipe_data_gnt, pipe_instr_gnt;
  //Todo remove gnt that is not used for returned resp delayed
  obi_req_t                     core_instr_req_ff;
  logic     [NCYCLES-1:0]       core_instr_resp_ff_rvalid;
  logic     [NCYCLES-1:0][31:0] core_instr_resp_ff_rdata;
  //Todo remove gnt that is not used for returned resp delayed
  obi_req_t                     core_data_req_ff;
  logic     [NCYCLES-1:0]       core_data_resp_ff_rvalid;
  logic     [NCYCLES-1:0][31:0] core_data_resp_ff_rdata;

  logic     [NCYCLES-1:0]       debug_req_ff;
  logic     [NCYCLES-1:0][31:0] intr_ff;


  //muxed master
  always_comb begin
    if (mask == 3'b011) begin
      mux_core_instr_req_i  = core_instr_req_i;
      core_instr_req_o      = mux_core_instr_req_o;
      mux_core_instr_resp_i = core_instr_resp_i;
      core_instr_resp_o     = mux_core_instr_resp_o;

      mux_core_data_req_i   = core_data_req_i;
      core_data_req_o       = mux_core_data_req_o;
      mux_core_data_resp_i  = core_data_resp_i;
      core_data_resp_o      = mux_core_data_resp_o;

      mux_debug_req_i       = debug_i;
      debug_o               = mux_debug_req_o;
      mux_intr_i            = intr_i;
      intr_o                = mux_intr_o;
    end else if (mask == 3'b110) begin
      //Instruction
      mux_core_instr_req_i[2]  = core_instr_req_i[0];
      core_instr_req_o[0]      = mux_core_instr_req_o[2];
      mux_core_instr_resp_i[2] = core_instr_resp_i[0];
      core_instr_resp_o[0]     = mux_core_instr_resp_o[2];


      mux_core_instr_req_i[0]  = core_instr_req_i[1];
      core_instr_req_o[1]      = mux_core_instr_req_o[0];
      mux_core_instr_resp_i[0] = core_instr_resp_i[1];
      core_instr_resp_o[1]     = mux_core_instr_resp_o[0];

      mux_core_instr_req_i[1]  = core_instr_req_i[2];
      core_instr_req_o[2]      = mux_core_instr_req_o[1];
      mux_core_instr_resp_i[1] = core_instr_resp_i[2];
      core_instr_resp_o[2]     = mux_core_instr_resp_o[1];

      //Data
      mux_core_data_req_i[2]   = core_data_req_i[0];
      core_data_req_o[0]       = mux_core_data_req_o[2];
      mux_core_data_resp_i[2]  = core_data_resp_i[0];
      core_data_resp_o[0]      = mux_core_data_resp_o[2];


      mux_core_data_req_i[0]   = core_data_req_i[1];
      core_data_req_o[1]       = mux_core_data_req_o[0];
      mux_core_data_resp_i[0]  = core_data_resp_i[1];
      core_data_resp_o[1]      = mux_core_data_resp_o[0];

      mux_core_data_req_i[1]   = core_data_req_i[2];
      core_data_req_o[2]       = mux_core_data_req_o[1];
      mux_core_data_resp_i[1]  = core_data_resp_i[2];
      core_data_resp_o[2]      = mux_core_data_resp_o[1];

      mux_debug_req_i[2]       = debug_i[0];
      debug_o[0]               = mux_debug_req_o[2];

      mux_debug_req_i[0]       = debug_i[1];
      debug_o[1]               = mux_debug_req_o[0];

      mux_debug_req_i[1]       = debug_i[2];
      debug_o[2]               = mux_debug_req_o[1];

      mux_intr_i[2]            = intr_i[0];
      intr_o[0]                = mux_intr_o[2];

      mux_intr_i[0]            = intr_i[1];
      intr_o[1]                = mux_intr_o[0];

      mux_intr_i[1]            = intr_i[2];
      intr_o[2]                = mux_intr_o[1];
    end else if (mask == 3'b101) begin
      //Instruction 
      mux_core_instr_req_i[0]  = core_instr_req_i[0];
      core_instr_req_o[0]      = mux_core_instr_req_o[0];
      mux_core_instr_resp_i[0] = core_instr_resp_i[0];
      core_instr_resp_o[0]     = mux_core_instr_resp_o[0];

      mux_core_instr_req_i[2]  = core_instr_req_i[1];
      core_instr_req_o[1]      = mux_core_instr_req_o[2];
      mux_core_instr_resp_i[2] = core_instr_resp_i[1];
      core_instr_resp_o[1]     = mux_core_instr_resp_o[2];

      mux_core_instr_req_i[1]  = core_instr_req_i[2];
      core_instr_req_o[2]      = mux_core_instr_req_o[1];
      mux_core_instr_resp_i[1] = core_instr_resp_i[2];
      core_instr_resp_o[2]     = mux_core_instr_resp_o[1];

      //Data
      mux_core_data_req_i[0]   = core_data_req_i[0];
      core_data_req_o[0]       = mux_core_data_req_o[0];
      mux_core_data_resp_i[0]  = core_data_resp_i[0];
      core_data_resp_o[0]      = mux_core_data_resp_o[0];

      mux_core_data_req_i[2]   = core_data_req_i[1];
      core_data_req_o[1]       = mux_core_data_req_o[2];
      mux_core_data_resp_i[2]  = core_data_resp_i[1];
      core_data_resp_o[1]      = mux_core_data_resp_o[2];

      mux_core_data_req_i[1]   = core_data_req_i[2];
      core_data_req_o[2]       = mux_core_data_req_o[1];
      mux_core_data_resp_i[1]  = core_data_resp_i[2];
      core_data_resp_o[2]      = mux_core_data_resp_o[1];

      mux_debug_req_i[0]       = debug_i[0];
      debug_o[0]               = mux_debug_req_o[0];

      mux_debug_req_i[2]       = debug_i[1];
      debug_o[1]               = mux_debug_req_o[2];

      mux_debug_req_i[1]       = debug_i[2];
      debug_o[2]               = mux_debug_req_o[1];

      mux_intr_i[0]            = intr_i[0];
      intr_o[0]                = mux_intr_o[0];

      mux_intr_i[2]            = intr_i[1];
      intr_o[1]                = mux_intr_o[2];

      mux_intr_i[1]            = intr_i[2];
      intr_o[2]                = mux_intr_o[1];

    end else begin
      mux_core_instr_req_i  = core_instr_req_i;
      core_instr_req_o      = mux_core_instr_req_o;
      mux_core_instr_resp_i = core_instr_resp_i;
      core_instr_resp_o     = mux_core_instr_resp_o;

      mux_core_data_req_i   = core_data_req_i;
      core_data_req_o       = mux_core_data_req_o;
      mux_core_data_resp_i  = core_data_resp_i;
      core_data_resp_o      = mux_core_data_resp_o;

      mux_debug_req_i       = debug_i;
      debug_o               = mux_debug_req_o;
      mux_intr_i            = intr_i;
      intr_o                = mux_intr_o;
    end
  end


  for (genvar i = 0; i < NHARTS; i++) begin : Nharts_delayed_mux

    if (i == 0) begin
      if (NCYCLES == 1) begin
        // Instruction
        obi_sngreg obi_sngreg0_i (
            .clk_i,
            .rst_ni,
            .clear_pipeline       (~enable_i),
            .core_instr_req_i     (mux_core_instr_req_i[0]),
            .core_instr_req_o     (core_instr_req_ff),
            .core_instr_resp_gnt_i(mux_core_instr_resp_i[0].gnt),
            .core_instr_resp_gnt_o(pipe_instr_gnt)
        );

        // Data
        obi_sngreg obi_sngreg1_i (
            .clk_i,
            .rst_ni,
            .clear_pipeline       (~enable_i),
            .core_instr_req_i     (mux_core_data_req_i[0]),
            .core_instr_req_o     (core_data_req_ff),
            .core_instr_resp_gnt_i(mux_core_data_resp_i[0].gnt),
            .core_instr_resp_gnt_o(pipe_data_gnt)
        );

      end else begin
        obi_pipelined_delay #(
            .NDELAY(NCYCLES)
        ) obi_pipelined_delay0_i (
            .clk_i,
            .rst_ni,
            .clear_pipeline       (~enable_i),
            .core_instr_req_i     (mux_core_instr_req_i[0]),
            .core_instr_req_o     (core_instr_req_ff),
            .core_instr_resp_gnt_i(mux_core_instr_resp_i[0].gnt),
            .core_instr_resp_gnt_o(pipe_instr_gnt)
        );

        // Data
        obi_pipelined_delay #(
            .NDELAY(NCYCLES)
        ) obi_pipelined_delay1_i (
            .clk_i,
            .rst_ni,
            .clear_pipeline       (~enable_i),
            .core_instr_req_i     (mux_core_data_req_i[0]),
            .core_instr_req_o     (core_data_req_ff),
            .core_instr_resp_gnt_i(mux_core_data_resp_i[0].gnt),
            .core_instr_resp_gnt_o(pipe_data_gnt)
        );
      end
    end

    always_comb begin
      //bypass by default 
      mux_core_instr_req_o[i]  = mux_core_instr_req_i[i];
      mux_core_instr_resp_o[i] = mux_core_instr_resp_i[i];
      mux_core_data_req_o[i]   = mux_core_data_req_i[i];
      mux_core_data_resp_o[i]  = mux_core_data_resp_i[i];

      mux_intr_o[i]            = mux_intr_i[i];
      mux_debug_req_o[i]       = mux_debug_req_i[i];

      if (enable_i) begin
        if (i == 0) begin
          mux_core_instr_req_o[0]         = core_instr_req_ff;
          mux_core_instr_resp_o[0].rdata  = mux_core_instr_resp_i[0].rdata;
          mux_core_instr_resp_o[0].rvalid = mux_core_instr_resp_i[0].rvalid;
          mux_core_instr_resp_o[0].gnt    = pipe_instr_gnt;
          mux_core_data_req_o[0]          = core_data_req_ff;
          mux_core_data_resp_o[0].rdata   = mux_core_data_resp_i[0].rdata;
          mux_core_data_resp_o[0].rvalid  = mux_core_data_resp_i[0].rvalid;
          mux_core_data_resp_o[0].gnt     = pipe_data_gnt;

          mux_intr_o[0]                   = mux_intr_i[0];
          mux_debug_req_o[0]              = mux_debug_req_i[0];
        end else if (i == 1) begin
          mux_core_instr_req_o[1]         = mux_core_instr_req_i[1];
          mux_core_instr_resp_o[1].rdata  = core_instr_resp_ff_rdata[NCYCLES-1];
          mux_core_instr_resp_o[1].rvalid = core_instr_resp_ff_rvalid[NCYCLES-1];
          mux_core_instr_resp_o[1].gnt    = mux_core_instr_resp_i[1].gnt;
          mux_core_data_req_o[1].addr     = mux_core_data_req_i[1].addr;
          mux_core_data_req_o[1].req      = mux_core_data_req_i[1].req;
          mux_core_data_req_o[1].be       = mux_core_data_req_i[1].be;
          mux_core_data_req_o[1].wdata    = mux_core_data_req_i[1].wdata;
          // when the buffered [0] req is granted the we is put to '0' in the output, while the output of [1] still one
          mux_core_data_req_o[1].we       = mux_core_data_req_i[1].we & mux_core_data_req_i[1].req;

          mux_core_data_resp_o[1].rdata   = core_data_resp_ff_rdata[NCYCLES-1];
          mux_core_data_resp_o[1].rvalid  = core_data_resp_ff_rvalid[NCYCLES-1];
          mux_core_data_resp_o[1].gnt     = mux_core_data_resp_i[1].gnt;
          /**/
          mux_intr_o[1]                   = intr_ff[NCYCLES-1];
          mux_debug_req_o[1]              = debug_req_ff[NCYCLES-1];
        end else begin

          mux_core_instr_req_o[i]  = mux_core_instr_req_i[i];
          mux_core_instr_resp_o[i] = mux_core_instr_resp_i[i];
          mux_core_data_req_o[i]   = mux_core_data_req_i[i];
          mux_core_data_resp_o[i]  = mux_core_data_resp_i[i];

          mux_intr_o[i]            = mux_intr_i[i];
          mux_debug_req_o[i]       = mux_debug_req_i[i];

        end
      end
    end
  end
  for (genvar j = 0; j < NCYCLES; j++) begin : N_Cycles_ff
    // Delayed Signals CPU ports
    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ndelay
      if (~rst_ni) begin
        intr_ff[j]                   <= '0;
        debug_req_ff[j]              <= '0;
        core_instr_resp_ff_rvalid[j] <= '0;
        core_instr_resp_ff_rdata[j]  <= '0;
        core_data_resp_ff_rvalid[j]  <= '0;
        core_data_resp_ff_rdata[j]   <= '0;
      end else begin
        if (j == 0) begin
          intr_ff[0]                   <= mux_intr_i[1];
          debug_req_ff[0]              <= mux_debug_req_i[1];

          core_instr_resp_ff_rvalid[0] <= mux_core_instr_resp_i[1].rvalid;
          core_instr_resp_ff_rdata[0]  <= mux_core_instr_resp_i[1].rdata;
          core_data_resp_ff_rvalid[0]  <= mux_core_data_resp_i[1].rvalid;
          core_data_resp_ff_rdata[0]   <= mux_core_data_resp_i[1].rdata;
        end else begin
          debug_req_ff[j]              <= debug_req_ff[j-1];
          intr_ff[j]                   <= intr_ff[j-1];

          core_instr_resp_ff_rvalid[j] <= core_instr_resp_ff_rvalid[j-1];
          core_instr_resp_ff_rdata[j]  <= core_instr_resp_ff_rdata[j-1];
          core_data_resp_ff_rvalid[j]  <= core_data_resp_ff_rvalid[j-1];
          core_data_resp_ff_rdata[j]   <= core_data_resp_ff_rdata[j-1];

        end
      end
    end
  end



endmodule  // lockstep_reg
