#!/bin/bash 

#PI=3.14
IDF_DIR=/home/jrm/v5.3.1/esp-idf/
PROJ_DIR=$(pwd)
echo "exporting paths for vscode"
cd $IDF_DIR
export IDF_PATH=$IDF_DIR IDF_TOOLS_PATH="/home/jrm/esp_v5_tools"
. ./export.sh
cd $PROJ_DIR
idf.py fullclean
#echo "Bye for now $LOGNAME. The time is `date +%T`!"
#now=$(date +"%m/%d/%y")
