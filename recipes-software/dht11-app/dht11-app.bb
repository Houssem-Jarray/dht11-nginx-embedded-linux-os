SUMMARY = "DHT11 application"
DESCRIPTION = "User application to read DHT11 values"
LICENSE = "CLOSED"

SRC_URI = "file://dht11-app/dht11_app.c"

S = "${WORKDIR}/dht11-app"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} dht11_app.c -o dht11_app
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 dht11_app ${D}${bindir}/
}