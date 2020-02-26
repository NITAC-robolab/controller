@ECHO OFF
"C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe" -S "P:\my_avrassem\2011robo\rcvr\labels.tmp" -fI -W+ie -C V2E -o "P:\my_avrassem\2011robo\rcvr\rcvr.hex" -d "P:\my_avrassem\2011robo\rcvr\rcvr.obj" -e "P:\my_avrassem\2011robo\rcvr\rcvr.eep" -m "P:\my_avrassem\2011robo\rcvr\rcvr.map" "P:\my_avrassem\2011robo\rcvr\rcvr.asm"
