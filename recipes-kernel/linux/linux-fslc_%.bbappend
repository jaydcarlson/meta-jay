PV = "4.0+git${SRCPV}"

#Make sure to only include these files if we're building MERT images
FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_remove_gallant = "git://github.com/Freescale/linux-fslc.git;branch=${SRCBRANCH}"

SRC_URI_append_gallant = " \
		git://git@github.com/Staticlabs/gallant-linux-4.0.0.git;protocol=ssh;branch=master \
		file://git/arch/arm/boot/dts/imx6dl-gallant.dts \
		file://0001-Improve-fxos8700-for-easier-access-to-values.patch \
		file://0001-Add-gpio-referenc.patch \
		file://0001-Added-support-for-system-serial-number.patch \
		file://defconfig \
		"
SRCREV = "${AUTOREV}"				
