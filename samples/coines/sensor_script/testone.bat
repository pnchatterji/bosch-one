@echo off
:: Brief: Flash and Run a built example to AB3 and dump results in console
"C:\Program Files (x86)\SEGGER\JLink\JRun.exe" -device NRF52840_xxAA --rtt On -if SWD --silent --exit "*STOP*" --semihost Off build\zephyr\zephyr.elf
