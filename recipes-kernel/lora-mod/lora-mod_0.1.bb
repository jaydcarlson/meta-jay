SUMMARY = "Lora Modem Driver"
LICENSE = "commercial"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r0"
PV = "0.3"

SRC_URI = "file://Makefile \
           file://COPYING \
           file://deferred.c \
           file://deferred.h \
           file://lmic_hal.c \
           file://lora_raw.c \
           file://lora_raw_chardev.c \
           file://lora_raw.h \
           file://lora_timer.c \
           file://lora_timer.h \
           file://lora_wan.c \
           file://lora_wan.h \
           file://lora_wan_packet.c \
           file://lora_wan_packet.h \
           file://lmic/aes.c \
           file://lmic/hal.h \
           file://lmic/lmic.h \
           file://lmic/lorabase.h \
           file://lmic/oslmic.h \
           file://lmic/radio.c \
           file://lmic/radio.h \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
