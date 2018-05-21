include recipes-core/images/rpi-hwup-image.bb
LICENSE = "MIT"
# IMAGE_FEATURES += "ssh-server-dropbear "

IMAGE_INSTALL += " nano linux-firmware hostapd iw wpa-supplicant dnsmasq "

DISTRO_FEATURES_append += " wifi"

ENABLE_UART = "1"