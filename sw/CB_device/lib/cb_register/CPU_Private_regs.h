// Generated register defines for CPU_Private

// Copyright information found in source file:
// Copyright lowRISC contributors.

// Licensing information found in source file:
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef _CPU_PRIVATE_REG_DEFS_
#define _CPU_PRIVATE_REG_DEFS_

#ifdef __cplusplus
extern "C" {
#endif
// Register width
#define CPU_PRIVATE_PARAM_REG_WIDTH 32

// Core_Id
#define CPU_PRIVATE_CORE_ID_REG_OFFSET 0x0
#define CPU_PRIVATE_CORE_ID_CORE_ID_MASK 0x7
#define CPU_PRIVATE_CORE_ID_CORE_ID_OFFSET 0
#define CPU_PRIVATE_CORE_ID_CORE_ID_FIELD \
  ((bitfield_field32_t) { .mask = CPU_PRIVATE_CORE_ID_CORE_ID_MASK, .index = CPU_PRIVATE_CORE_ID_CORE_ID_OFFSET })

// ACK of Sync Interrupt
#define CPU_PRIVATE_HART_INTC_ACK_REG_OFFSET 0x4
#define CPU_PRIVATE_HART_INTC_ACK_HART_INTC_ACK_BIT 0

// Breakpoint_Sim
#define CPU_PRIVATE_BREAKPOINT_SIM_REG_OFFSET 0x8
#define CPU_PRIVATE_BREAKPOINT_SIM_BREAKPOINT_BIT 0

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // _CPU_PRIVATE_REG_DEFS_
// End generated register defines for CPU_Private