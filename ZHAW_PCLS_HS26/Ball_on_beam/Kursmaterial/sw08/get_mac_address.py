"""
With the code below, we
"""

import network
import ubinascii

# Enable WLAN
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# Get MAC address and reformat
mac = network.WLAN().config("mac")
mac = ubinascii.hexlify(mac, ":").decode()
print(mac)
