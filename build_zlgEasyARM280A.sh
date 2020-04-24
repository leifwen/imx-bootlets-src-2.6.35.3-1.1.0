export PATH=$PATH:/em_cc_sc/project/i.mx28_EasyARM280A/cc/cc_for_bootlets/bin/
rm /em_cc_sc/project/i.mx28_EasyARM280A/root/nfspath/imx28_ivt_uboot.sb
rm ./imx28_ivt_uboot.sb
make BOARD=iMX28_EVK clean
make BOARD=iMX28_EVK CROSS_COMPILE=arm-fsl-linux-gnueabi-
cp ./imx28_ivt_uboot.sb /em_cc_sc/project/i.mx28_EasyARM280A/root/nfspath
