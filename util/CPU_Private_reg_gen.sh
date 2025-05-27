echo "Generating RTL"
${PYTHON} ../../esl_epfl_x_heep/hw/vendor/pulp_platform_register_interface/vendor/lowrisc_opentitan/util/regtool.py -r -t ../rtl ../data/CPU_Private_reg.hjson
mv ../rtl/cpu_private_reg_pkg.sv ../rtl/include
echo "Generating SW"
${PYTHON} ../../esl_epfl_x_heep/hw/vendor/pulp_platform_register_interface/vendor/lowrisc_opentitan/util/regtool.py --cdefines -o ../sw/CB_device/lib/cb_register/CPU_Private_regs.h ../data/CPU_Private_reg.hjson
