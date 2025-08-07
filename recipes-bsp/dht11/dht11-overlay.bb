SUMMARY = "DHT11 Overlay Recipe"
DESCRIPTION = "Builds device tree overlay for DHT11 sensor"
LICENSE = "CLOSED"

inherit devicetree deploy

SRC_URI = "file://dht11.dts"
S = "${WORKDIR}"

DTB_NAME = "dht11.dtbo"

do_compile() {
    echo "Compiling DHT11 overlay..."

    dtc -@ -I dts -O dtb \
        -i ${STAGING_KERNEL_DIR}/include \
        -i ${S} \
        -o ${S}/${DTB_NAME} \
        ${S}/dht11.dts
}

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/overlays
    install -m 0644 ${S}/${DTB_NAME} ${D}${nonarch_base_libdir}/firmware/overlays/
}

do_deploy() {
    install -d ${DEPLOYDIR}/overlays
    install -m 0644 ${S}/${DTB_NAME} ${DEPLOYDIR}/overlays/
}

FILES:${PN} = "${nonarch_base_libdir}/firmware/overlays/${DTB_NAME}"

COMPATIBLE_MACHINE = "raspberrypi4-64"
