#!/bin/bash

CALL_DIR=$(pwd)

SRC_DIR=$1
echo $1

CMSIS_DIR=$SRC_DIR/Drivers/CMSIS
HAL_DIR=$SRC_DIR/Drivers/STM32F7xx_HAL_Driver

CMSIS_INC=$CMSIS_DIR/Include
CMSIS_LIB=$CMSIS_DIR/Lib/GCC
CMSIS_DEV=$CMSIS_DIR/Device/ST/STM32F7xx

CMSIS_DEV_INC=$CMSIS_DEV/Include
CMSIS_DEV_STARTUP=$CMSIS_DEV/Source/Templates/gcc

HAL_INC=$HAL_DIR/Inc
HAL_SRC=$HAL_DIR/Src

echo "Moving to output directory..."
cd "$(dirname "$0")"
echo -e "\tRemoving old CMSIS Files..."
rm -rf cmsis_inc cmsis_lib cmsis_dev_inc cmsis_dev_startup
echo -e "\tCopying in new CMSIS Files..."
cp -r $CMSIS_INC/ cmsis_inc
cp -r $CMSIS_LIB/ cmsis_lib
cp -r $CMSIS_DEV_INC cmsis_dev_inc
cp -r $CMSIS_DEV_STARTUP cmsis_dev_startup
echo -e "\tRemoving old HAL Files..."
rm -rf hal_inc hal_src
echo -e "\tCopying in new HAL Files..."
cp -r $HAL_INC/ hal_inc
cp -r $HAL_SRC hal_src
echo "Returning to call directory..."
cd $CALL_DIR