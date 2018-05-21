DESCRIPTION = "Add firmware for bcm4339"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${THISDIR}/brcm4339-firmware/license;md5=d218b86dc92a1519dd093fe84c407d9e"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "file://brcmfmac4339-sdio.txt \
                file://brcmfmac4339-sdio.bin"

do_compile() {
    :
}

do_install() {
    install -d ${D}${base_libdir}/firmware
    install -d ${D}${base_libdir}/firmware/brcm
    cp ${WORKDIR}/brcmfmac4339-sdio.bin ${D}${base_libdir}/firmware/brcm/
    cp ${WORKDIR}/brcmfmac4339-sdio.txt ${D}${base_libdir}/firmware/brcm/
}

FILES_${PN} = "/lib/firmware/brcm/*"
