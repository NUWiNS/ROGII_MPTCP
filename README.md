# ROGII_MPTCP

Additional Files required to compile this kernel:

1. android-ndk-r21d
2. clang r383902b

COMPILING MPTCP KERNEL

1. If you are compiling for the first time, run the build_kirisakura_clean.sh script
  
      a. You have to modify the CLANG path & Cross Compiler path in the script
  
2. After successfull compilation got to the AnyKernel Directory

     a. Remove any existing kernel.zip
  
     b. Create a directory structure as such - modules/system/vendor/lib/modules
  
     c. run the copy_modules.sh script 
  
     d. Zip up the kernel with zip_up_kernel.sh script: a kernel.zip will be created
  
FLASHING MPTCP KERNEL
  
3. Before flashing the Kirisakura kernel, make sure your phone is rooted with the required stock firmware and Magisk manager is installed

     a. adb connect to the phone and do adb reboot bootloader
  
     b. Use twrp to flash the kernel. Get the twrp image from https://drive.google.com/file/d/1L6HOdcdwXMB-HbUG8NpbsJHB-mA3re5b/view?usp=sharing
  
     c. Once the bootloader screen is seen, enter the following commands: Fastboot boot <twrp image file name>
  
     d. Once twrp recovery is up, you will have to install the custom kernel. Make sure you have the kernel zip in the phone’s sd card. Click on install and 
        select the kernel zip. Swipe the bar at bottom to install. 
  
     e. Reboot to bootloader again and boot to twrp using fastboot and flash the kernel again. Reboot to system now. 
  
     f. Custom kernel is installed. 
