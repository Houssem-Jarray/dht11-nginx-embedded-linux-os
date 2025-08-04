SUMMARY = "User-space GPIO access library"
DESCRIPTION = "Simple C shared library to control GPIO"
LICENSE = "CLOSED"
SRC_URI = "file://rpi_gpio/src/gpio.c \
           file://rpi_gpio/include/gpio.h"

S = "${WORKDIR}/rpi_gpio"
inherit pkgconfig

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} -Wall -fPIC -c ${S}/src/gpio.c -I${S}/include -o gpio.o
    ${CC} ${LDFLAGS} -shared -Wl,-soname,librpi_gpio.so.1 -o librpi_gpio.so.1.0 gpio.o
    ln -sf librpi_gpio.so.1.0 librpi_gpio.so
    ln -sf librpi_gpio.so.1.0 librpi_gpio.so.1
}

do_install() {
    install -d ${D}${libdir}
    install -m 0755 librpi_gpio.so.1.0 ${D}${libdir}/
    ln -sf librpi_gpio.so.1.0 ${D}${libdir}/librpi_gpio.so.1
    ln -sf librpi_gpio.so.1.0 ${D}${libdir}/librpi_gpio.so

    install -d ${D}${includedir}
    install -m 0644 ${S}/include/gpio.h ${D}${includedir}/
}
