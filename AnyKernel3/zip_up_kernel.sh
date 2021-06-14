#!/bin/bash

#KERNEL_BASE=$1

echo "copying kernel image here"
cp /data/imran/Kirisakura/Kirisakura_mptcp_1/out/arch/arm64/boot/Image.gz-dtb ./

#echo "copying modules here"
./copy_modules.sh /data/imran/Kirisakura/Kirisakura_mptcp_1

echo "zipping up kernel"
zip -r9 kernel.zip * -x .git *placeholder README.md kernel.zip zip_up_kernel.sh

echo "deleting kernel image"
rm Image.gz-dtb

echo "done"
