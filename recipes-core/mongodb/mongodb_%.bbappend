FILESEXTRAPATHS_append := ":${THISDIR}/files"

SRC_URI_append = " \
    file://mongo.conf \
    file://mongod \
    "

do_install_append() {
        install -m 0744 -D ${WORKDIR}/mongod ${D}${sysconfdir}/init.d
        install -m 0744 -D ${WORKDIR}/mongo.conf ${D}${sysconfdir}
}