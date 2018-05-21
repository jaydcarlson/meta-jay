DESCRIPTION = "Gateway image file"

IMAGE_INSTALL = "packagegroup-core-boot ${ROOTFS_PKGMANAGE_BOOTSTRAP} ${CORE_IMAGE_EXTRA_INSTALL}"

IMAGE_LINGUAS = " "

LICENSE = "MIT"

inherit core-image

IMAGE_ROOTFS_SIZE = "8192"

IMAGE_FEATURES += "dev-pkgs debug-tweaks eclipse-debug"
IMAGE_INSTALL += " linux-firmware kernel-modules iw sen-test brcm4339-firmware dr-test wpa-supplicant git ntp ntpdate ntp-utils "
IMAGE_INSTALL += "lora-mod"
IMAGE_INSTALL += "i2c-tools"
IMAGE_INSTALL += "ppp"
IMAGE_INSTALL += "links"
IMAGE_INSTALL += "extrafsfiles"
IMAGE_INSTALL += "evtest"
IMAGE_INSTALL += "mono mozroot-certdata"
IMAGE_INSTALL += "cronie"
IMAGE_INSTALL += "alsa-utils"
IMAGE_INSTALL += "raindetection"
IMAGE_INSTALL += "watchdog"
IMAGE_INSTALL += "solvzwatchdog"

MACHINE_FEATURES += "alsa"
IMAGE_INSTALL += "alsa-lib"

# ROOTFS_POSTPROCESS_COMMAND += "set_root_passwd;"
set_root_passwd() {
   sed 's%^root:[^:]*:%root:$1$ppQRxtqs$t8f31ZfCVgjoIOzSBAQPF0:%' \
       < ${IMAGE_ROOTFS}/etc/passwd \
       > ${IMAGE_ROOTFS}/etc/passwd.new;
   mv ${IMAGE_ROOTFS}/etc/passwd.new ${IMAGE_ROOTFS}/etc/passwd ;
}
