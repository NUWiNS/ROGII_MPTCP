# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() { '
kernel.string=[wil6210 modified Wigig MTU mod 1] Kirisakura-Kernel for Asus Rog Phone 2 aka Yoda by freak07 @ xda-developers
do.devicecheck=0
do.modules=1
do.systemless=1
do.cleanup=1
do.cleanuponabort=0
device.name1=
device.name2=
device.name3=
device.name4=
device.name5=
supported.versions=10
supported.patchlevels=2020-07 -
'; } # end properties

# shell variables
block=/dev/block/bootdevice/by-name/boot;
is_slot_device=1;
ramdisk_compression=auto;


## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;


## AnyKernel file attributes
# set permissions/ownership for included ramdisk files
set_perm_recursive 0 0 755 644 $ramdisk/*;
set_perm_recursive 0 0 750 750 $ramdisk/init* $ramdisk/sbin;

## begin vendor changes
mount -o rw,remount -t auto /vendor >/dev/null;

cp -rf /tmp/anykernel/patch/mixer_paths_ZS660KL_EU.xml /vendor/etc/mixer_paths_ZS660KL_EU.xml;
set_perm 0 0 0644 /vendor/etc/mixer_paths_ZS660KL_EU.xml;

cp -rf /tmp/anykernel/patch/msm_irqbalance.conf /vendor/etc/msm_irqbalance.conf;
set_perm 0 0 0644 /vendor/etc/msm_irqbalance.conf;

# Make a backup of init.target.rc
restore_file /vendor/etc/init/hw/init.target.rc;


## AnyKernel install
dump_boot;

# Clean up other kernels' ramdisk overlay files
rm -rf $ramdisk/overlay;
rm -rf $ramdisk/overlay.d;

# begin ramdisk changes

# remove old root patch avoidance hack
patch_cmdline "skip_override" "";

#F2FS Optimization (anykernel implementation by @kdrag0n)
if mountpoint -q /data; then
  # Optimize F2FS extension list (@arter97)
  for list_path in $(find /sys/fs/f2fs* -name extension_list); do
    hash="$(md5sum $list_path | sed 's/extenstion/extension/g' | cut -d' ' -f1)"

    # Skip update if our list is already active
    if [[ $hash == "43df40d20dcb96aa7e8af0e3d557d086" ]]; then
      echo "Extension list up-to-date: $list_path"
      continue
    fi

    ui_print "  • Optimizing F2FS extension list"
    echo "Updating extension list: $list_path"

    echo "Clearing extension list"

    hot_count="$(grep -n 'hot file extens' $list_path | cut -d':' -f1)"
    list_len="$(cat $list_path | wc -l)"
    cold_count="$((list_len - hot_count))"

    cold_list="$(head -n$((hot_count - 1)) $list_path | grep -v ':')"
    hot_list="$(tail -n$cold_count $list_path)"

    for ext in $cold_list; do
      [ ! -z $ext ] && echo "[c]!$ext" > $list_path
    done

    for ext in $hot_list; do
      [ ! -z $ext ] && echo "[h]!$ext" > $list_path
    done

    echo "Writing new extension list"

    for ext in $(cat $home/f2fs-cold.list | grep -v '#'); do
      [ ! -z $ext ] && echo "[c]$ext" > $list_path
    done

    for ext in $(cat $home/f2fs-hot.list); do
      [ ! -z $ext ] && echo "[h]$ext" > $list_path
    done
  done
fi

# Overlay
if [ -d $ramdisk/.backup ]; then
	mv /tmp/anykernel/overlay.d $ramdisk/overlay.d
	chmod -R 750 $ramdisk/overlay.d/*
	chown -R root:root $ramdisk/overlay.d/*
	chmod -R 755 $ramdisk/overlay.d/sbin/init.kirisakura.sh
	chown -R root:root $ramdisk/overlay.d/sbin/init.kirisakura.sh
fi;

# end ramdisk changes

write_boot;
## end install

	ui_print " "; ui_print "[wil6210 modified Wigig working MTU mod 1]Kirisakura-Kernel successfully flashed. Enjoy your device and have a nice day!";

