# **RB5 Hotspot Config**

## **Connect to robot with wire**
- 连接C口
- ``adb devices``
- ``adb root``
- ``adb shell``
- ``su``

## **Hotspot**
- 可进入robot terminal后用``nmcli con show`` or ``ifconfig``查看
- 在**wlan0**
- SSID: RobotWu
- Pass: wuzhenyu

## **Wifi**
- 可在进入robot terminal后用``nmcli con show`` or ``ifconfig``查看
- 在**ap0**, 会将wifi共享给所有连接RB5 Hotspot的设备
- SSID：SpectrumSetup-F9
- Pass: quickcanoe091

## **ssh**
- 连接到RB5的hotspot
- ``ssh root@10.42.0.1``
- pass: oelinux123
