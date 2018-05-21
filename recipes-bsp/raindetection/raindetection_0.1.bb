SUMMARY = "Audio rain detection"
LICENSE = "CLOSED"

PACKAGE_ARCH = "${MACHINE_ARCH}"

PR = "r1"

BRANCH = "master"
SRCREV = "${AUTOREV}"

GIT_URI = "git://git@github.com/Staticlabs/rain-detection-app.git"
GIT_PROTOCOL = "ssh"

SRC_URI = "${GIT_URI};protocol=${GIT_PROTOCOL};branch=${BRANCH} \
           file://raincheck \
"

S = "${WORKDIR}/git/"

RDEPENDS_${PN} = "libasound"

CXXFLAGS += "-std=c++11"

do_compile() {
     	oe_runmake
}

do_install() {
	install -d ${D}${prefix}/local/rain
	install -d ${D}${sysconfdir}/init.d
	install -m 0755 ${S}/RainDetectorGallant ${D}${prefix}/local/rain/
	install -m 0755 ${WORKDIR}/raincheck ${D}${sysconfdir}/init.d/
}

FILES_${PN} = "${prefix}/local/*"
FILES_${PN} += "${sysconfdir}/*"
FILES_${PN}-dbg += "${prefix}/local/rain/.debug"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
