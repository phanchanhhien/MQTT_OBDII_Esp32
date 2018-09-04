ECHO OFF
ECHO ESP32 debug 
decoder.py -p ESP32 -t ~/.platformio/packages/toolchain-xtensa32 -e .pioenvs/esp32dev/firmware.elf myStackTrace.txt
PAUSE
