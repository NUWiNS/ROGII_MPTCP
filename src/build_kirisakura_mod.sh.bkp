#!/bin/bash
# Use if have built the kernel before and want to compile the kernel after making some changes (avoids building kernel from scratch)

echo
echo "Issue Build Commands"
echo

export ARCH=arm64
export SUBARCH=arm64
export CLANG_PATH=/data/imran/Kirisakura/linux-x86/clang-r383902b/bin
export PATH=${CLANG_PATH}:${PATH}
export CLANG_TRIPLE=aarch64-linux-gnu-
export CROSS_COMPILE=/data/moinak/Kirisakura/android-ndk-r21d/toolchains/aarch64-linux-android-4.9/prebuilt/linux-x86_64/bin/aarch64-linux-android-
export CROSS_COMPILE_ARM32=/data/moinak/Kirisakura/android-ndk-r21d/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-
export LD_LIBRARY_PATH=/data/imran/Kirisakura/linux-x86/clang-r383902b/lib64:$LD_LIBRARY_PATH

echo
echo "Build The Good Stuff"
echo 

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip O=out -j9
