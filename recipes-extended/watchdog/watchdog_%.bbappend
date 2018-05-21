FILESEXTRAPATHS_append := "${THISDIR}/${PN}"

SRC_URI_append_gallant = " \
   file://0001-Add-delay-to-init.patch \
"
