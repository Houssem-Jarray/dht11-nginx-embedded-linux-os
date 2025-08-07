SUMMARY = "DHT11 Character Device Driver"
DESCRIPTION = "A simple DHT11 character device driver (no device tree)"
LICENSE = "CLOSED"

SRC_URI = "file://dht11driver/dht11driver.c \
           file://dht11driver/Makefile"

S = "${WORKDIR}/dht11driver"

inherit module

EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"

do_compile() {
    make -C ${STAGING_KERNEL_DIR} M=${S} modules
}

do_install() {
    # Install the kernel module
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/dht11driver.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
    
    # Install modules-load.d config for auto-loading at boot
    install -d ${D}${sysconfdir}/modules-load.d
    echo "dht11driver" > ${D}${sysconfdir}/modules-load.d/dht11driver.conf
    
    # Install udev rule to set device permissions
    install -d ${D}${sysconfdir}/udev/rules.d
    echo 'KERNEL=="dht11", MODE="0666"' > ${D}${sysconfdir}/udev/rules.d/99-dht11.rules
}

FILES:${PN} += "${sysconfdir}/modules-load.d/dht11driver.conf"
FILES:${PN} += "${sysconfdir}/udev/rules.d/99-dht11.rules"

COMPATIBLE_MACHINE = "raspberrypi4-64"