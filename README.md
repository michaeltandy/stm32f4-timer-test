Make with: `make && openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg -c "program build/Test1.elf verify reset exit"`

Debug with `openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg 1>/dev/null 2>&1& arm-none-eabi-gdb --eval-command="target remote localhost:3333" build/Test1.elf; killall openocd`
