SUMMARY = "I2C drivers test"
DECSRIPTION = "Copy the driver test files to the rootfs"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${THISDIR}/dr-test/test_drivers;beginline=3;endline=6;md5=3dd153bcd9fe5eed8d87d3060b379bc3"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "file://test_drivers \
		file://test_si7006 \
		file://test_fxos8700 \
		file://test_bmp280 \
		file://test_ltr303 \
		file://test_veml6070 \
		file://test_stc3115"

do_compile() {
    :
}

do_install() {
	install -d ${D}${prefix}/local
	install -d ${D}${prefix}/local/bin
	install -d ${D}${prefix}/local/driver-test
	install -m 0755 ${WORKDIR}/test_drivers ${D}${prefix}/local/bin/
	install -m 0755 ${WORKDIR}/test_si7006 ${D}${prefix}/local/driver-test/
	install -m 0755 ${WORKDIR}/test_fxos8700 ${D}${prefix}/local/driver-test/
	install -m 0755 ${WORKDIR}/test_bmp280 ${D}${prefix}/local/driver-test/
	install -m 0755 ${WORKDIR}/test_ltr303 ${D}${prefix}/local/driver-test/
	install -m 0755 ${WORKDIR}/test_veml6070 ${D}${prefix}/local/driver-test/
	install -m 0755 ${WORKDIR}/test_stc3115 ${D}${prefix}/local/driver-test/
}

FILES_${PN} = "/usr/local/*"
