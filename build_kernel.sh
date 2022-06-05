#!/bin/bash

export ARCH=arm64
export ANDROID_MAJOR_VERSION=q
make exynos9810-crownlte_defconfig
make -j64
