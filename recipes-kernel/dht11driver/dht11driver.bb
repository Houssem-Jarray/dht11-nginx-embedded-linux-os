SUMMARY = "DHT11 Kernel Driver"
DESCRIPTION = "A DHT11 character kernel module device driver"
LICENSE = "CLOSED"
PACKAGE_NAME = "${PN}-${PV}"

SRC_URI = "file://dht11driver/dht11driver.c \
           file://dht11driver/Makefile"

           
S = "${WORKDIR}/dht11driver"
inherit module

EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"

do_compile() {
    make -C ${STAGING_KERNEL_DIR} M=${S} modules
}

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/dht11driver.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
}
