DESCRIPTION = "Simple Hello Kernel Module with /dev/hello"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = "file://hello.c \
           file://Makefile \
           file://hello.conf \
"

S = "${WORKDIR}"

inherit module

EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"

do_compile() {
    oe_runmake -C ${STAGING_KERNEL_DIR} M=${S} modules
}

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/hello.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/

    install -d ${D}${sysconfdir}/modules-load.d
    install -m 0644 ${WORKDIR}/hello.conf ${D}${sysconfdir}/modules-load.d/hello.conf
}

FILES:${PN} += "${sysconfdir}/modules-load.d/hello.conf"
