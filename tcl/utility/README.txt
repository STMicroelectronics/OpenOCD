this folder contains openocd scripts to full erase flash memory and check memory is fully erased


eg. to fully erase flash memory of spc582b (Chorus 1M):

open a dos command prompt and run following command:

C:\StellarLINK\openocd\bin\openocd.exe -d0 -s C:\StellarLINK\openocd\scripts -f utility\spc582b_dis_FLASH_ERASE_ALL.cfg

to check that all memory has been cleared use the following command

C:\StellarLINK\openocd\bin\openocd.exe -d0 -s C:\StellarLINK\openocd\scripts -f utility\spc582b_dis_FLASH_ERASE_CHECK.cfg



to flash an spc582b elf program the following command can be used:

C:\StellarLINK\openocd\bin\openocd.exe -d0 -s C:\StellarLINK\openocd\scripts -f board\spc582b_dis.cfg -c  "program blinkled_demo.elf reset exit"


be careful to use the right script for the device you are using
above command don't apply for elf file compiled to run from RAM


if your openocd installation come from SPC5Studio or StellarStudio the path above must be modified as follow

C:\SPC5Studio-6.0\openocd\bin\openocd.exe -d0 -s C:\SPC5Studio-6.0\openocd\scripts -f utility\spc582b_dis_FLASH_ERASE_ALL.cfg

C:\SPC5Studio-6.0\openocd\bin\openocd.exe -d0 -s C:\SPC5Studio-6.0\openocd\scripts -f board\spc582b_dis.cfg -c  "program blinkled_demo.elf reset exit"

or

C:\StellarStudio-2.0\openocd\bin\openocd.exe -d0 -s C:\StellarStudio-2.0\openocd\scripts -f utility\sr5e1_evbe7000p_FLASH_ERASE_ALL.cfg

C:\StellarStudio-2.0\openocd\bin\openocd.exe -d0 -s C:\StellarStudio-2.0\openocd\scripts -f board\sr5e1_evbe7000p.cfg -c  "program blinkled_portE13.elf reset exit"

