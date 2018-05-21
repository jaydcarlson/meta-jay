SUMMARY = "Solvz Watchdog test scripts"
LICENSE = "CLOSED"

PACKAGE_ARCH = "${MACHINE_ARCH}"

PR = "r0"


SRC_URI = "file://processcheck \
           file://restartprocess \
           file://checkmodemconnection \
           file://watchdogtest.h \
           file://watchdogtest.c \
           file://Makefile \
           file://SolvzWatchdog.make \
           file://checkweatherappcrash \
"

S = "${WORKDIR}/"


do_compile() {
        oe_runmake
}

do_install() {
        install -d ${D}${prefix}/local/watchdog
        install -d ${D}${sysconfdir}/rc5.d
	install -d ${D}${sysconfdir}/watchdog.d
        install -m 0755 ${S}/SolvzWatchdogGallant ${D}${sysconfdir}/watchdog.d/
        install -m 0755 ${WORKDIR}/processcheck ${D}${prefix}/local/watchdog/
	install -m 0755 ${WORKDIR}/restartprocess ${D}${prefix}/local/watchdog/
	install -m 0755 ${WORKDIR}/checkweatherappcrash ${D}${prefix}/local/watchdog/
	install -m 0755 ${WORKDIR}/checkmodemconnection ${D}${prefix}/local/watchdog/
}

FILES_${PN} = "${prefix}/local/*"
FILES_${PN} += "${sysconfdir}/*"
FILES_${PN}-dbg += "${prefix}/local/watchdog/.debug"
FILES_${PN}-dbg += "${sysconfdir}/watchdog.d/.debug"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

