DESCRIPTION = "Add files to be used as startup scripts and misc config"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${THISDIR}/extrafsfiles/license;md5=d218b86dc92a1519dd093fe84c407d9e"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI ="file://wpa_supplicant.conf \
		file://startwifi \
		file://gemalto \
		file://cell-init.sh \
		file://gallant \
		file://gallant.sh \
		file://intellifarm.sh \
		file://checkwifi \
		file://access-gps \
		file://check-ppp \
		file://log-data \
		file://update-time \
		file://set-gpio \
		file://led-control \
		file://ledinit \
		file://led-manager \
		file://checkmodem \
                file://startwatchdog \
"
inherit update-rc.d
INITSCRIPT_NAME = "ledinit"
INITSCRIPT_PARAMS = "start 99 1 2 3 4 5 . stop 99 0 6 ."

do_compile() {
    :
}

do_install() {
    install -d ${D}${prefix}/local/wifi
    install -d ${D}${prefix}/local/cellular
    install -d ${D}${sysconfdir}/init.d
    install -d ${D}${sysconfdir}/rc5.d
    install -d ${D}${prefix}/local/bin
    install -m 0777 ${WORKDIR}/wpa_supplicant.conf ${D}${prefix}/local/wifi/
    install -m 0755 ${WORKDIR}/startwifi ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/gemalto ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/cell-init.sh ${D}${prefix}/local/cellular/init.sh
    install -m 0755 ${WORKDIR}/gallant ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/gallant.sh ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/checkwifi ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/intellifarm.sh ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/checkmodem ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/access-gps ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/check-ppp ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/log-data ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/update-time ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/set-gpio ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/led-control ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/ledinit ${D}${sysconfdir}/init.d/
    install -m 0755 ${WORKDIR}/led-manager ${D}${prefix}/local/bin/
    install -m 0755 ${WORKDIR}/startwatchdog ${D}${sysconfdir}/init.d/
    ln -s ../init.d/gemalto ${D}${sysconfdir}/rc5.d/S20gemalto
    ln -s ../init.d/ppp ${D}${sysconfdir}/rc5.d/S80ppp
    ln -s ../init.d/gallant ${D}${sysconfdir}/rc5.d/S99gallant
    
}

#ln -s ../init.d/clear-led ${D}${sysconfdir}/rc5.d/S99zclear-led

FILES_${PN} = "${prefix}/local/*"
FILES_${PN} += "${sysconfdir}/*"

pkg_postinst_extrafsfiles() {
    test -d $D${localstatedir}/spool/cron || mkdir -p $D${localstatedir}/spool/cron
    if ! grep -q -s update-time $D${localstatedir}/spool/cron/root; then
        echo "adding crontab"
        echo "0 * * * *    ${prefix}/local/bin/update-time" >> $D${localstatedir}/spool/cron/root
    fi
    if ! grep -q -s check-ppp $D${localstatedir}/spool/cron/root; then
        echo "adding crontab"
        echo "0 * * * *    ${prefix}/local/bin/check-ppp" >> $D${localstatedir}/spool/cron/root
    fi
    if ! grep -q -s log-data $D${localstatedir}/spool/cron/root; then
        echo "adding crontab"
        echo "0-59/5 * * * *    ${prefix}/local/bin/log-data" >> $D${localstatedir}/spool/cron/root
    fi
}
