FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " file://wpa_supplicant.conf"

do_install:append() {
    install -d ${D}${sysconfdir}/wpa_supplicant
    install -m 600 ${WORKDIR}/wpa_supplicant.conf ${D}${sysconfdir}/wpa_supplicant/wpa_supplicant.conf
}
