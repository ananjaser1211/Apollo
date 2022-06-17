#!/bin/bash
#
# Apollo Build Script V1.0
# For Exynos9810
# Forked from Exynos8890 Script
# Coded by AnanJaser1211 @ 2019-2021
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software

# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Main Dir
CR_DIR=$(pwd)
# Define proper arch and dir for dts files
CR_DTS=arch/arm64/boot/dts/exynos
# Define boot.img out dir
CR_OUT=$CR_DIR/Apollo/Out
CR_PRODUCT=$CR_DIR/Apollo/Product
# Presistant A.I.K Location
CR_AIK=$CR_DIR/Apollo/A.I.K
# Main Ramdisk Location
CR_RAMDISK=$CR_DIR/Apollo/Ramdisk
# Compiled image name and location (Image/zImage)
CR_KERNEL=$CR_DIR/arch/arm64/boot/Image
# Compiled dtb by dtbtool
CR_DTB=$CR_DIR/arch/arm64/boot/dtb.img
# Kernel Name and Version
CR_VERSION=V2.0
CR_NAME=Apollo
# Thread count
CR_JOBS=$(nproc --all)
# Target Android version
CR_ANDROID=q
CR_PLATFORM=10.0.0
# Target ARCH
CR_ARCH=arm64
# Current Date
CR_DATE=$(date +%Y%m%d)
# General init
export ANDROID_MAJOR_VERSION=$CR_ANDROID
export PLATFORM_VERSION=$CR_PLATFORM
export $CR_ARCH
##########################################
# Device specific Variables [SM-G960X]
CR_CONFIG_G960=starlte_defconfig
CR_VARIANT_G960=G960X
# Device specific Variables [SM-G965X]
CR_CONFIG_G965=star2lte_defconfig
CR_VARIANT_G965=G965X
# Device specific Variables [SM-N960X]
CR_CONFIG_N960=crownlte_defconfig
CR_VARIANT_N960=N960F
# Common configs
CR_CONFIG_9810=exynos9810_defconfig
CR_CONFIG_SPLIT=NULL
CR_CONFIG_APOLLO=apollo_defconfig
CR_PERMISSIVE="0"
# Compiler Paths
CR_GCC11=~/Android/Toolchains/aarch64-linux-gnu-11.x/bin/aarch64-linux-gnu-
CR_GCC9=~/Android/Toolchains/aarch64-linux-gnu-9.x/bin/aarch64-linux-gnu-
CR_CLANG=~/Android/Toolchains/clang-r353983c/bin
CR_GCC4=~/Android/Toolchains/aarch64-linux-android-4.9/bin/aarch64-linux-android-
#####################################################

# Compiler Selection
BUILD_COMPILER()
{
if [ $CR_COMPILER = "1" ]; then
export CROSS_COMPILE=$CR_GCC4
compile="make"
CR_COMPILER="$CR_GCC4"
fi
if [ $CR_COMPILER = "2" ]; then
export CLANG_PATH=$CR_CLANG
export CROSS_COMPILE=$CR_GCC11
export CLANG_TRIPLE=aarch64-linux-gnu-
compile="make CC=clang ARCH=arm64"
export PATH=${CLANG_PATH}:${PATH}
CR_COMPILER="$CR_CLANG"
fi
if [ $CR_COMPILER = "3" ]; then
export CROSS_COMPILE=$CR_GCC11
compile="make"
CR_COMPILER="$CR_GCC11"
fi
if [ $CR_COMPILER = "4" ]; then
export CROSS_COMPILE=$CR_GCC9
compile="make"
CR_COMPILER="$CR_GCC9"
fi
}

# Clean-up Function

BUILD_CLEAN()
{
if [ $CR_CLEAN = "y" ]; then
     echo " "
     echo " Cleaning build dir"
     $compile clean && $compile mrproper
     rm -r -f $CR_DTB
     rm -rf $CR_DTS/.*.tmp
     rm -rf $CR_DTS/.*.cmd
     rm -rf $CR_DTS/*.dtb
     rm -rf $CR_DIR/.config
     rm -rf $CR_OUT/*.img
     rm -rf $CR_OUT/*.zip
fi
if [ $CR_CLEAN = "n" ]; then
     echo " "
     echo " Skip Full cleaning"
     rm -r -f $CR_DTB
     rm -rf $CR_DTS/.*.tmp
     rm -rf $CR_DTS/.*.cmd
     rm -rf $CR_DTS/*.dtb
     rm -rf $CR_DIR/.config
     rm -rf $CR_DIR/.version
fi
}


# Kernel Name Function

BUILD_IMAGE_NAME()
{
	CR_IMAGE_NAME=$CR_NAME-$CR_VERSION-$CR_VARIANT-$CR_DATE
}

# Config Generation Function

BUILD_GENERATE_CONFIG()
{
  # Only use for devices that are unified with 2 or more configs
  echo "----------------------------------------------"
  echo "Building defconfig for $CR_VARIANT"
  echo " "
  # Respect CLEAN build rules
  BUILD_CLEAN
  if [ -e $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig ]; then
    echo " cleanup old configs "
    rm -rf $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig
  fi
  echo " Copy $CR_CONFIG "
  cp -f $CR_DIR/arch/$CR_ARCH/configs/$CR_CONFIG $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig
  # Split-config support for devices with unified defconfigs (Universal + device)
  if [ $CR_CONFIG_SPLIT = NULL ]; then
    echo " No split config support! "
  else
    echo " Copy $CR_CONFIG_SPLIT "
    cat $CR_DIR/arch/$CR_ARCH/configs/$CR_CONFIG_SPLIT >> $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig
  fi
  # Apollo Custom defconfig
  echo " Copy $CR_CONFIG_APOLLO "
  cat $CR_DIR/arch/$CR_ARCH/configs/$CR_CONFIG_APOLLO >> $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig
  # Selinux Never Enforce all targets
  if [ $CR_PERMISSIVE = "1" ]; then
    echo " Building Permissive Kernel"
    echo "CONFIG_ALWAYS_PERMISSIVE=y" >> $CR_DIR/arch/$CR_ARCH/configs/tmp_defconfig
  fi
  echo " Set $CR_VARIANT to generated config "
  CR_CONFIG=tmp_defconfig
}

# Kernel information Function
BUILD_OUT()
{
  echo " "
  echo "----------------------------------------------"
  echo "$CR_VARIANT kernel build finished."
  echo "Compiled DTB Size = $sizdT Kb"
  echo "Kernel Image Size = $sizT Kb"
  echo "Boot Image   Size = $sizkT Kb"
  echo "$CR_PRODUCT/$CR_IMAGE_NAME.img Ready"
  echo "Press Any key to end the script"
  echo "----------------------------------------------"
}

# Kernel Compile Function
BUILD_ZIMAGE()
{
	echo "----------------------------------------------"
	echo " "
	echo "Building zImage for $CR_VARIANT"
	export LOCALVERSION=-$CR_IMAGE_NAME
	echo "Make $CR_CONFIG"
	$compile $CR_CONFIG
	echo "Make Kernel with $CR_COMPILER"
	$compile -j$CR_JOBS
	if [ ! -e $CR_KERNEL ]; then
	exit 0;
	echo "Image Failed to Compile"
	echo " Abort "
	fi
	du -k "$CR_KERNEL" | cut -f1 >sizT
	sizT=$(head -n 1 sizT)
	rm -rf sizT
	echo " "
	echo "----------------------------------------------"
}

# Device-Tree compile Function
BUILD_DTB()
{
	echo "----------------------------------------------"
	echo " "
	echo "Checking DTB for $CR_VARIANT"
	# This source does compiles dtbs while doing Image
	if [ ! -e $CR_DTB ]; then
        exit 0;
        echo "DTB Failed to Compile"
        echo " Abort "
	else
        echo "DTB Compiled at $CR_DTB"
	fi
	rm -rf $CR_DTS/.*.tmp
	rm -rf $CR_DTS/.*.cmd
	rm -rf $CR_DTS/*.dtb
	du -k "$CR_DTB" | cut -f1 >sizdT
	sizdT=$(head -n 1 sizdT)
	rm -rf sizdT
	echo " "
	echo "----------------------------------------------"
}

# Ramdisk Function
PACK_BOOT_IMG()
{
	echo "----------------------------------------------"
	echo " "
	echo "Building Boot.img for $CR_VARIANT"
	# Copy Ramdisk
	cp -rf $CR_RAMDISK/* $CR_AIK
	# Move Compiled kernel and dtb to A.I.K Folder
	mv $CR_KERNEL $CR_AIK/split_img/boot.img-zImage
	mv $CR_DTB $CR_AIK/split_img/boot.img-dtb
	# Create boot.img
	$CR_AIK/repackimg.sh
	if [ ! -e $CR_AIK/image-new.img ]; then
        exit 0;
        echo "Boot Image Failed to pack"
        echo " Abort "
	fi
	# Remove red warning at boot
	echo -n "SEANDROIDENFORCE" » $CR_AIK/image-new.img
	# Copy boot.img to Production folder
	if [ ! -e $CR_PRODUCT ]; then
        mkdir $CR_PRODUCT
	fi
	cp $CR_AIK/image-new.img $CR_PRODUCT/$CR_IMAGE_NAME.img
	# Move boot.img to out dir
	if [ ! -e $CR_OUT ]; then
        mkdir $CR_OUT
	fi
	mv $CR_AIK/image-new.img $CR_OUT/$CR_IMAGE_NAME.img
	du -k "$CR_OUT/$CR_IMAGE_NAME.img" | cut -f1 >sizkT
	sizkT=$(head -n 1 sizkT)
	rm -rf sizkT
	echo " "
	$CR_AIK/cleanup.sh
	# Respect CLEAN build rules
	BUILD_CLEAN
}

# Single Target Build Function
BUILD()
{
	if [ "$CR_TARGET" = "1" ]; then
		echo " Galaxy S9 "
		CR_CONFIG_SPLIT=$CR_CONFIG_G960
		CR_VARIANT=$CR_VARIANT_G960
	fi
	if [ "$CR_TARGET" = "2" ]; then
		echo " Galaxy S9+ "
		CR_CONFIG_SPLIT=$CR_CONFIG_G965
		CR_VARIANT=$CR_VARIANT_G965
	fi
	if [ "$CR_TARGET" = "3" ]
	then
		echo " Galaxy Note 9 "
		CR_CONFIG_SPLIT=$CR_CONFIG_N960
		CR_VARIANT=$CR_VARIANT_N960
	fi
	CR_CONFIG=$CR_CONFIG_9810
	CR_PERMISSIVE="0"
	BUILD_COMPILER
	BUILD_CLEAN
	BUILD_IMAGE_NAME
	BUILD_GENERATE_CONFIG
	BUILD_ZIMAGE
	BUILD_DTB
	PACK_BOOT_IMG
	BUILD_OUT
}

# Multi-Target Build Function
BUILD_ALL(){
echo "----------------------------------------------"
echo " Compiling ALL targets "
CR_TARGET=1
BUILD
CR_TARGET=2
BUILD
CR_TARGET=3
BUILD
}

# Main Menu
clear
echo "----------------------------------------------"
echo "$CR_NAME $CR_VERSION Build Script $CR_DATE"
echo " "
echo " "
echo "1) starlte" "2) star2lte" "3) crownlte" "4) All" "5) Abort"
echo "----------------------------------------------"
read -p "Please select your build target (1-4) > " CR_TARGET
echo "----------------------------------------------"
echo " "
echo "1) $CR_GCC4 (GCC 4.9)"
echo "2) $CR_CLANG (CLANG)" 
echo "3) $CR_GCC11 (GCC 11.x)" 
echo "4) $CR_GCC9 (GCC 9.x)" 
echo " "
read -p "Please select your compiler (1-4) > " CR_COMPILER
if [ "$CR_TARGET" = "5" ]; then
echo "Build Aborted"
exit
fi
echo " "
read -p "Clean Builds? (y/n) > " CR_CLEAN
echo " "
# Call functions
if [ "$CR_TARGET" = "4" ]; then
BUILD_ALL
else
BUILD
fi
