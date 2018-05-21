include recipes-core/images/rpi-hwup-image.bb
LICENSE = "MIT"
IMAGE_FEATURES += "ssh-server-dropbear "

IMAGE_INSTALL += " nano linux-firmware oracle-jse-jre mongodb hostapd iw wpa-supplicant dnsmasq dosfstools libusb1 "

DISTRO_FEATURES_append += " wifi"

ENABLE_UART = "1"