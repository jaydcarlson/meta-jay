FILESEXTRAPATHS_append := "${THISDIR}/${PN}"

SRC_URI_append_imx6ul-breakout = " \
   file://0001-add-support-for-imx6ul-breakout.patch \
"
