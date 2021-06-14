KERNEL_BASE=$1

mkdir -p ./modules/system/vendor/lib/modules/

cp $KERNEL_BASE/out/net/bridge/br_netfilter.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/ec_hid_driver/ec_i2c_interface.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/ec_hid_driver/ec_i2c_updatefw.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/aura_sync/ene_6k582_station.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/dt_aura/ene_8k41_dt.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/dt_power/ene_8k41_power.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/usb/host/ghci/ghci-hcd.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/media/usb/gspca/gspca_main.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/usb/host/ghci/gxhci-hcd.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/aura_sync/ml51fb9ae_inbox.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/media/platform/msm/dvb/adapter/mpq-adapter.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/media/platform/msm/dvb/demux/mpq-dmx-hw-plugin.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/platform/msm/msm_11ad/msm_11ad_proxy.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/media/rc/msm-geni-ir.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/fan/nct7802.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/char/rdbg.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/techpack/data/rmnet/perf/rmnet_perf.ko  ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/techpack/data/rmnet/shs/rmnet_shs.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/input/touchscreen/station_goodix_gtx8/station_goodix_touch.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/ec_hid_driver/station_key.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/media/platform/msm/broadcast/tspp.ko ./modules/system/vendor/lib/modules/
cp $KERNEL_BASE/out/drivers/net/wireless/ath/wil6210/wil6210.ko ./modules/system/vendor/lib/modules/
