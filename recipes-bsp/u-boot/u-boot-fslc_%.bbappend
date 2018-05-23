FILESEXTRAPATHS_append := "${THISDIR}/${PN}"

# SRC_URI_remove = "git://github.com/Freescale/u-boot-fslc.git;branch=${SRCBRANCH}"

SRC_URI = "git:///home/jay/u-boot-imx6ul;protocol=file;branch=mx6ul-breakout"

SRCREV = "${AUTOREV}"