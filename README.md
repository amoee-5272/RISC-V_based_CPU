# RISC-V_based_CPU
111-2 Computer Architecture Final project

## start  
cd ../01_RTL/  <br>
source 00_license.f  <br>
source 01_run.f  <br>


## other test pattern  
vcs ../00_TB/tb.v CHIP.v -full64 -R -\ debug_access+all +v2k +notimingcheck +define+I0   <br>
// The line in 01_run.f can be chosen from I0 to I3


