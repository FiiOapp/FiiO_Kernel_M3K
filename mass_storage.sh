echo "building mass storage kernel ..."
cd $BUILD_ROOT_PATH/kernel/
make halley2_linux_sfcnand_mass_defconfig
make xImage
make modules
cp arch/mips/boot/compressed/xImage $BUILD_ROOT_PATH/out/product/halley2/image/xImage_TF_card
cp drivers/usb/gadget/g_mass_storage.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/libcomposite.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/usb_f_mass_storage.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/u_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/usb_f_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/usb_f_acm.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/g_serial.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/usb/gadget/g_audio.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/usb/gadget/
cp drivers/staging/dwc2/dwc2.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/staging/dwc2/
cp drivers/staging/dwc2/fiio_wakeup.ko $BUILD_ROOT_PATH/out/product/halley2/system/lib/modules/3.10.14/kernel/driver/staging/dwc2/
rm arch/mips/boot/compressed/xImage
rm arch/mips/boot/zcompressed/xImage
mkfs.ubifs -e 0x1f000 -c 2048 -m 0x800 -d $BUILD_ROOT_PATH/out/product/halley2/system -o $BUILD_ROOT_PATH/out/product/halley2/image/system.ubi
make halley2_linux_sfcnand_ubi_defconfig

