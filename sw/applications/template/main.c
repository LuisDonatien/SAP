// Copyright 2025 CEI UPM
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Luis Waucquez (luis.waucquez.jimenez@upm.es)
  
#include <stdio.h>
#include <stdlib.h>
#include "csr.h"
#include "csr_registers.h"
#include "CB_Safety.h"


int main(int argc, char *argv[]) 
{

        /******START******/

        /*
        
        **Insert program**
        
        */

        //Enter Safe mode (TCLS_MODE DCLS_MODE LOCKSTEP_MODE)
        Safe_Activate(DCLS_MODE);

        /*
        
        **Insert program**
        
        */

        //Checkpoint for DCLS configuration
        Store_Checkpoint();

        /*
        
        **Insert program**
        
        */

        //Exit Safe mode (MASTER_CORE0 MASTER_CORE1 MASTER_CORE2)
        Safe_Stop(MASTER_CORE1); 

        /*
        
        **Insert program**
        
        */

        /******END PROGRAM******/
    
        return 0;
}