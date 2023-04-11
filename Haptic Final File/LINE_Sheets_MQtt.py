from machine import I2C,Pin
import time
import network
import ujson as json

# 1. prepare LED and sensors
    
# 1. prepare WiFi
# https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
wlan_if = network.WLAN(network.STA_IF)
wlan_if.active(False)
time.sleep(1)
wlan_if.active(True)
wlan_if.connect('POCO X3 NFC', '0815513255')
while not wlan_if.isconnected():
    print('Connecting WiFi')
    time.sleep(1)
print(wlan_if.ifconfig())

# 2. install required libraries
import upip
upip.install('urequests')
upip.install('umqtt.simple')
import urequests as requests
import umqtt.simple as mqtt

# 4. receive msg
def mqtt_cb(topic, msg):
    print(topic, msg)

if __name__ == '__main__':
    client = mqtt.MQTTClient("vsupacha_xxx", "broker.hivemq.com")
    client.connect()
    while True:
        msg = {'id':'#1234', 'status':'start'}
        client.publish("thammasat/vsupacha/mbits/btn", json.dumps(msg))
        client.check_msg()
        time.sleep(2)