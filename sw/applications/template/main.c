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

        //Enter Safe mode (TMR_MODE DMR_MODE LOCKSTEP_MODE)
        TMR_Safe_Activate(DMR_MODE);

        /*
        
        **Insert program**
        
        */

        //Checkpoint for DMR configuration
        Store_Checkpoint();

        /*
        
        **Insert program**
        
        */

        //Exit Safe mode (MASTER_CORE0 MASTER_CORE1 MASTER_CORE2)
        TMR_Safe_Stop(MASTER_CORE1); 

        /*
        
        **Insert program**
        
        */

        /******END PROGRAM******/
    
        return 0;
}