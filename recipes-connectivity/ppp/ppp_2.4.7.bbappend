#Make sure to only include these files if we're building MERT images
FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_gallant = " \
                            file://options \
                            file://net-connect \
                            file://net-chat \
                            file://verizon \
                            file://ppp_on_boot \
                            "

do_install_append () {
	install -m 0644 ${WORKDIR}/options ${D}${sysconfdir}/ppp/
	install -m 0755 ${WORKDIR}/net-connect ${D}${sysconfdir}/ppp/
	install -m 0644 ${WORKDIR}/net-chat ${D}${sysconfdir}/ppp/
	install -m 0644 ${WORKDIR}/verizon ${D}${sysconfdir}/ppp/peers/verizon
	install -m 0755 ${WORKDIR}/ppp_on_boot ${D}${sysconfdir}/ppp/ppp_on_boot
}
