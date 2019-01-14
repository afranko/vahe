#!/bin/bash

# A fajlt masold annak a projekt mappajaba, amelyet leforditottal ( a monitor es a buildflash csak itt mukodik )
# pelda.: sh prog.sh -b -m -f -c 6    -> ekkor akarok buildelni es flashelni, akarok monitort, akarok flasht is
# ( ami folosleges, mmert a -b paranccsal ez mar lement ), es a 6-os portra tortenjen mindez
# masodik pelda: sh prog.sh -f -a C:/User/You/Documents/esp/actual_project -> ekkor az altalad megadott mappaba bemasolt
# 3 fajlt onnan fogja az ESP-re felflashelni. a 3 fajl neve es elerese az ANOTHER_DIR sornal olvashato. ha a -a parancs aktiv,
# nem mukodik a -b es a -m


BUILD_FLASH="0"	##if u want to build and flash the ESP 				-b
MONITOR="0"		##if u want to start monitor after flash 			-m
FLASH="0"		##if u want to flash the board with the built files -f
COM_NUMBER="0"	##the serial port number of the ESP. its required if its another board, or u want to flash only -c COMX where X is the number
ANOTHER_DIR="0"	##if u want to flash the script from another directory -a C:/User/You/Documents/esp/actual_project
				## in this case, u just have to copy these 3 files from the original directory to the new: 	build/bootloader/bootloader.bin
				##																							build/uart_echo.bin
				##																							build/partitions_singleapp.bin





while getopts bmfc:a: option
do
case "${option}"
in
b) 	BUILD_FLASH="1";;
m) 	MONITOR="1";;
f) 	FLASH="1";;
c)	COM_NUMBER=${OPTARG};;
a)	ANOTHER_DIR=${OPTARG};;
esac
done

if [ "$COM_NUMBER" != "0" ]
then
	COM_NUMBER="COM"$COM_NUMBER
fi

##DEBUG PRINT
if [ "$BUILD_FLASH" = "1" ]
then
	echo "Flasheles es buildeles kivalasztva"
fi
if [ "$MONITOR" = "1" ]
then
	echo "Monitor kivalasztva"
fi
if [ "$FLASH" = "1" ]
then
	echo "Flasheles kivalasztva"
fi
if [ "$COM_NUMBER" != "0" ]
then
	echo "A beolvasott soros port jelzese $COM_NUMBER"
fi
if [ "$ANOTHER_DIR" != "0" ]
then
	echo "$ANOTHER_DIR"
fi

##DEBUG PRINT END


SED_PARAMETER='s/^(CONFIG_ESPTOOLPY_PORT[[:blank:]]*=[[:blank:]]*).*/\1'
SED_PARAMETER+='"'
SED_PARAMETER+=$COM_NUMBER
SED_PARAMETER+='"/'

KEY="CONFIG_ESPTOOLPY_PORT"
FILE="sdkconfig"


if [ "$BUILD_FLASH" = "1" ] && [ "$ANOTHER_DIR" = "0" ] && [ "$COM_NUMBER" != "0" ]
then
	$echo sed -E -i $SED_PARAMETER sdkconfig
fi
if [ "$MONITOR" = "1" ] && [ "$ANOTHER_DIR" = "0" ] && [ "$COM_NUMBER" != "0" ]
then
	$echo sed -E -i $SED_PARAMETER sdkconfig
fi

if [ "$BUILD_FLASH" = "1" ] && [ "$ANOTHER_DIR" = "0" ] && [ "$COM_NUMBER" != "0" ]
then
	$echo sed -E -i $SED_PARAMETER sdkconfig.old
fi
if [ "$MONITOR" = "1" ] && [ "$ANOTHER_DIR" = "0" ] && [ "$COM_NUMBER" != "0" ]
then
	$echo sed -E -i $SED_PARAMETER sdkconfig.old
fi

if [ "$COM_NUMBER" = "0" ]
then
	COM_NUMBER=$(grep "^$KEY" $FILE | cut -d'=' -f2)
fi
echo "Az irando soros port jelzese : $COM_NUMBER"

if [ "$BUILD_FLASH" = "1" ] && [ "$ANOTHER_DIR" = "0" ]
then
	$echo make flash
elif [ "$FLASH" = "1" ] && [ "$ANOTHER_DIR" = "0" ]
then
	$echo python C:/msys32/home/heged/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port $COM_NUMBER --baud 115200 --before "default_reset" --after "hard_reset" write_flash -z --flash_mode "dio" --flash_freq "40m" --flash_size detect   0x1000 build/bootloader/bootloader.bin 0x10000  build/aflegacy_vahe.bin 0x8000 build/partitions_singleapp.bin
elif [ "$FLASH" = "1" ]
then
	$echo python C:/msys32/home/heged/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port $COM_NUMBER --baud 115200 --before "default_reset" --after "hard_reset" write_flash -z --flash_mode "dio" --flash_freq "40m" --flash_size detect   0x1000 $ANOTHER_DIR/bootloader.bin 0x10000  $ANOTHER_DIR/uart_echo.bin 0x8000 $ANOTHER_DIR/partitions_singleapp.bin
fi

if [ "$MONITOR" = "1" ] && [ "$ANOTHER_DIR" = "0" ]
then
	$echo make monitor
fi
