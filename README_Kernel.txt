################################################################################
1. How to Build
        - get Toolchain
                From android git server, codesourcery and etc ..
                - gcc-cfp/gcc-cfp-jopp-only/aarch64-linux-android-4.9/bin/aarch64-linux-android-
        - edit Makefile
                edit "CROSS_COMPILE" to right toolchain path(You downloaded).
                        EX)  CROSS_COMPILE=<android platform directory you download>/android/prebuilts/gcc-cfp/gcc-cfp-jopp-only/aarch64-linux-android-4.9/bin/aarch64-linux-android-
                        EX)  CROSS_COMPILE=/usr/local/toolchain/gcc-cfp/gcc-cfp-jopp-only/aarch64-linux-android-4.9/bin/aarch64-linux-android- // check the location of toolchain
        - to Build
                $ export PLATFORM_VERSION=11
                $ export ANDROID_MAJOR_VERSION=r
                $ export ARCH=arm64
                $ make exynos9810-r7_defconfig
                $ make

2. Output files
        - Kernel : arch/arm64/boot/Image
        - module : drivers/*/*.ko

3. How to Clean
        $ make clean
################################################################################
