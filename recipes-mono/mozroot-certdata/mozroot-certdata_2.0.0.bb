FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"

SUMMARY = "Mono Mozilla Root Certificates"

LICENSE = "MPL-1.1"
LIC_FILES_CHKSUM = "file://../certdata.txt;md5=b5b009a1c475f7bb95ac8c55f80442f3"

#
# Note that this local file is taken from the default mozroots source URI here
#
# http://mxr.mozilla.org/seamonkey/source/security/nss/lib/ckfw/builtins/certdata.txt?raw=1
#
# There are newer certdata.txt files available. These do not seem to import cleanly with
# older versions of Mono (e.g. 3.12.1) but do seem to import with Mono 4.0.2
#
# see: http://curl.haxx.se/mail/archive-2013-12/0033.html
#

SRC_URI = "file://certdata.txt file://runmozroots"

SRC_URI[md5sum] = "b5b009a1c475f7bb95ac8c55f80442f3"
SRC_URI[sha256sum] = "ea89ac8ae495e69586abae22941816842ca5811a32a20dc9e1adb95859802879"

do_install_append() {
 install -d "${D}${sysconfdir}"
 install -d "${D}${sysconfdir}/ssl"
 install -d "${D}${sysconfdir}/init.d"
 install -d "${D}${sysconfdir}/rc5.d"
 install -m 644 ${S}/../certdata.txt ${D}${sysconfdir}/ssl/certdata.txt
 install -m 755 ${S}/../runmozroots ${D}${sysconfdir}/init.d/runmozroots
 ln -s ../init.d/runmozroots ${D}${sysconfdir}/rc5.d/S20runmozroots
}

FILES_${PN} = "${sysconfdir}/ssl/certdata.txt ${sysconfdir}/init.d/runmozroots ${sysconfdir}/rc5.d/S20runmozroots"

inherit allarch

