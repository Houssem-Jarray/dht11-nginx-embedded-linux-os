#!/bin/sh

echo "🔄 Reconnecting Wi-Fi..."

rfkill unblock wlan
ip link set wlan0 down
killall wpa_supplicant 2>/dev/null || true
sleep 1
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
sleep 2
ip link set wlan0 up

# Use udhcpc (from busybox) instead of dhclient
echo "🌐 Requesting IP address with udhcpc..."
udhcpc -i wlan0
