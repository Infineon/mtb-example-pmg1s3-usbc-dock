#!/bin/bash
# post build script for combining bootloader and applicaton firmware.
# Script creates bootloader application image (.cyacd2) files.

#echo off

# param 1 - cymcuelf tool path
export TOOL_PATH=$1

# param 2 - binary build path
export BUILD_DIR=$2

# param 3 - application extension (fw1/fw2).
export APP_EXTN=$3

# param 4 - application file name without extension.
export APP_NAME=$4

# param 5 - Bootloader name
export BOOT_NAME=$5

# param 6 - MTB Tools path
export MTB_TOOLS=$6

# param 7 - OS information
export OS_INPUT=$7

echo TOOL_PATH=$TOOL_PATH BUILD_DIR=$BUILD_DIR APP_EXTN=$APP_EXTN MTB_TOOLS=$MTB_TOOLS OS_INPUT=$OS_INPUT 

# Select bin2psochex converter based on OS type
if [ $OS_INPUT = 'Windows_NT' ]
then
    export BIN2PSOCHEX=./bin2psochex.exe
    export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat.exe
    export PYTHON=$MTB_TOOLS/python/python.exe
else 
	OS_NAME=$(uname -s)
	if [ $OS_NAME = 'Linux' ]
	then
            export BIN2PSOCHEX=./bin2psochex_linux
	    export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat
            export PYTHON=python3
	else # Default to MAC OS
            export BIN2PSOCHEX=./bin2psochex_mac
	    export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat
            export PYTHON=python3
	fi
fi

# Sign an create boot loadable .cyacd2 file 
$TOOL_PATH --sign $APP_NAME'.elf' CRC --output $APP_NAME'_'$APP_EXTN'.elf'
$TOOL_PATH -P $APP_NAME'_'$APP_EXTN'.elf' --output $APP_NAME'_'$APP_EXTN'.cyacd2'


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
# Build Firmware2
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
build_fw2() {
    #Delete objects files from the previous build
    rm -rf $BUILD_DIR/bsps $BUILD_DIR/ext $BUILD_DIR/src
    rm -f *.o *.d
    
    make APPNAME_EXT=fw2 build
    return
}

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#Merge bootloader and dual applicatoin firmware
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
merge_dual() {
	pushd utils

	#Get device sillicon id from the hex file placed at address offset 0x90500002
	#Binary format(B-Record): Freescale Dragonball bootstrap record format
	#4-byte address (big endian), a 1-byte length, and then data bytes as 
	#indictated by the length
	$SREC_CAT $APP_NAME'.hex' -Intel -crop 0x90500002 0x90500006 -offset -0x90500002 -o sid.bin -B-Record
	read sid<sid.bin
	sid=0x${sid:10:8}
	rm sid.bin
    
    #Remove existing hex and elf binary files.
	rm -f $APP_NAME'.elf' $APP_NAME'.hex'

    #Create composite hex file of Bootloader, FW1 and FW2
	$TOOL_PATH --merge ../bootstrap/$BOOT_NAME'.elf' \
					$APP_NAME'_fw1.elf' \
					$APP_NAME'_fw2.elf' \
					--output $APP_NAME'.elf' \
					--hex $APP_NAME'.hex'

	#Create a full image bin file from hex file
	$SREC_CAT $APP_NAME'.hex' -Intel -crop 0x0 0x40000 -o $APP_NAME'.bin' -binary
    
    #Enable write protection for bootloder flash area
	$BIN2PSOCHEX $sid 0x0F 0x400  $APP_NAME'.bin' $APP_NAME'_temp.hex'
    
    #Re-calculate combined HASH for application and metadata.
    $PYTHON hex_bin_update.py -i $APP_NAME'_temp.hex' -o $APP_NAME'.hex'

    #Remove temporary binary files.
    rm -f $APP_NAME'_temp.hex' $APP_NAME'_fw1.hex' $APP_NAME'_fw2.hex' $APP_NAME'_fw1.elf' $APP_NAME'_fw2.elf'

    popd
	return
}

# Build dual FW condition check
if [ $APP_EXTN = 'fw2' ]    
then
    merge_dual
else
    build_fw2
fi

exit
