#!/bin/bash
#You may have to change wlan0 to match whatever is in ifconfig. Register the device by MAC address on ncsu nomad
wlan_interface=wlan0
stop network-manager
printf 'network={\n\tssid="ncsu"\n\tkey_mgmt=NONE\n\tpriority=-999\n\t}' > /home/root/.temp.conf
wpa_supplicant -B -i$wlan_interface -c/home/root/.temp.conf -Dnl80211
printf 'wpa_supplicant done\n'
dhclient wlan0
printf 'dhclient done\n'
ntpdate -s time.nist.gov
rm /home/root/.temp.conf