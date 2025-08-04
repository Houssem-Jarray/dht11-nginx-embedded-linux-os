SUMMARY = "DHT11 Kernel Driver"
DESCRIPTION = "A DHT11 character kernel module device driver"
HOMEPAGE = ""
LICENSE = "CLOSED"

SRC_URI = "file://dht11/dht11.c \
           file://dht11/Makefile"


S = "${WORKDIR}/dht11"

inherit module

KERNEL_MODULE_AUTOLOAD +="dht11"