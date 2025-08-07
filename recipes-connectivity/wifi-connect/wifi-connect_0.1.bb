SUMMARY = "Auto Wi-Fi connect on boot"
DESCRIPTION = "Installs a Wi-Fi auto-connect script and systemd service"
LICENSE = "CLOSED"
SRC_URI = "file://connect.sh \
           file://wifi-connect.service"

S = "${WORKDIR}"

inherit systemd

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/connect.sh ${D}${bindir}/connect.sh

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/wifi-connect.service ${D}${systemd_system_unitdir}/wifi-connect.service
}

SYSTEMD_SERVICE:${PN} = "wifi-connect.service"
SYSTEMD_AUTO_ENABLE = "enable"
