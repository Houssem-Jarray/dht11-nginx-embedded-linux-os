FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://dht11.cfg"

# Make sure the DHT11 driver is available for the dht11driver recipe
PROVIDES += "virtual/dht11-kernel-support"