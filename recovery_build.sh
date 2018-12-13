echo "building recovery ..."
cd $BUILD_ROOT_PATH/kernel/
make halley2_sfcnand_recovery_defconfig
make recovery
cp arch/mips/boot/compressed/recovery $BUILD_ROOT_PATH/out/product/halley2/image/
rm arch/mips/boot/compressed/xImage
rm arch/mips/boot/zcompressed/xImage
make halley2_linux_sfcnand_ubi_defconfig

