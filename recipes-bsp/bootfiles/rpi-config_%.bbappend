# recipes-bsp/bootfiles/rpi-config_%.bbappend
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

RPI_EXTRA_CONFIG:append = "\n# DHT11 sensor overlay\n\
dtoverlay=dht11,gpiopin=4\n"
