DESCRIPTION = "gpsapp"
PR = "r0"
LICENSE = "CLOSED"
RM_WORK_EXCLUDE += "gpsapp"

SRC_URI = "file://main.c \
          "
do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} ${WORKDIR}/main.c -o gpsapp
}

do_install() {
    install -m 0755 -d ${D}${bindir}
    install -m 0755 ${S}/gpsapp ${D}${bindir}
}
