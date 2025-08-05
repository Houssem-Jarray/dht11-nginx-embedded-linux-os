SUMMARY = "DHT11 Overlay Recipe"
DESCRIPTION = "Builds device tree overlay for DHT11 sensor"
LICENSE = "CLOSED"

inherit devicetree deploy

SRC_URI = "file://dht11.dts"
S = "${WORKDIR}"

DTB_NAME = "dht11.dtbo"

do_compile() {
    echo "Compiling DHT11 overlay..."
    
    # Use dtc to compile the device tree overlay
    dtc -@ -I dts -O dtb \
        -i ${STAGING_KERNEL_DIR}/include \
        -i ${S} \
        -o ${S}/${DTB_NAME} \
        ${S}/dht11.dts
}

do_install() {
    install -d ${D}/boot/overlays/
    install -m 0644 ${S}/${DTB_NAME} ${D}/boot/overlays/
}

do_deploy() {
    install -d ${DEPLOYDIR}
    install -m 0644 ${S}/${DTB_NAME} ${DEPLOYDIR}/
}

# Use the correct variable name for newer Yocto versions
FILES:${PN} = "/boot/overlays/${DTB_NAME}"

COMPATIBLE_MACHINE = "raspberrypi4-64"