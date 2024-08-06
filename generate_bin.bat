@echo off
set /p version=Input version:
echo %version%
set scriptDir=%~dp0

call vivado -mode tcl -nojournal -nolog -source generate_mcs.tcl

xcopy %scriptDir%project_1\project_1.runs\impl_1\sfpga_top.bit %scriptDir%version_bin\
xcopy %scriptDir%project_1\project_1.runs\impl_1\sfpga_top.ltx %scriptDir%version_bin\
@REM xcopy %scriptDir%project_1\project_1.runs\impl_1\sfpga_top.bin %scriptDir%version_bin\
xcopy %scriptDir%project_1\project_1.runs\impl_1\sfpga_top.mcs %scriptDir%version_bin\

ren %scriptDir%version_bin\sfpga_top.bit PCG_TimingS_v%version%.bit 
ren %scriptDir%version_bin\sfpga_top.ltx PCG_TimingS_v%version%.ltx 
@REM ren %scriptDir%version_bin\sfpga_top.bin PCG_TimingS_v%version%.bin 
ren %scriptDir%version_bin\sfpga_top.mcs PCG_TimingS_v%version%.mcs 

pause