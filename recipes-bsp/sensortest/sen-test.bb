SUMMARY = "Sensor data upload"
DECSRIPTION = "Upload sensor data to the cloud"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${WORKDIR}/sensordatacheck;beginline=3;endline=6;md5=9aff7c9c5c6bce6c8a63d08123fcba6d"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "file://updatesensordata \
                file://sensordatacheck"

do_compile() {
    :
}

do_install() {
	install -d ${D}${prefix}/local
	install -d ${D}${prefix}/local/bin
        install -m 0755 ${WORKDIR}/updatesensordata ${D}${prefix}/local/bin/
        install -m 0755 ${WORKDIR}/sensordatacheck ${D}${prefix}/local/bin/
}

FILES_${PN} = "/usr/local/bin/*"
