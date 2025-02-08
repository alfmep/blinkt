# Linux driver for the Blinkt! RGB LED by Pimoroni.

A Linux device driver and device tree overlay for the [Blinkt! RGB LED by Pimoroni](https://shop.pimoroni.com/products/blinkt).

*See also the led application from project [ledpp](https://github.com/alfmep/ledpp).*

### Table of contents
- **[How to build and install](#how-to-build-and-install)**
  - [Getting the source code](#getting-the-source-code)
  - [Build the blinkt kernel module](#build-the-blinkt-kernel-module)
  - [Install the blinkt kernel module and device tree overlay](#install-the-blinkt-kernel-module-and-device-tree-overlay)
- **[How to use](#how-to-use)**
  - [Loading the kernel module](#loading-the-kernel-module)
  - [Controlling the LEDs from a command shell](#controlling-the-leds-from-a-command-shell)
  - [Set a LED trigger](#set-a-led-trigger)
  - [Initialize LEDs with module parameters](#initialize-leds-with-module-parameters)


## How to build and install
### Getting the source code
Open up a command terminal on your Raspberry Pi. Download the source code using git:
```shell
git clone https://github.com/alfmep/blinkt.git
```
### Build the blinkt kernel module

Go to the blinkt directory and build the kernel module and device tree overlay:
```shell
cd blinkt
make
```
If you get an error message that looks something like this:
```shell
bin/sh: 1: ./scripts/dtc/dtc: not found
```
Then build using the following commands:
```shell
make SKIP_OVERLAY=1
dtc -@ -Hepapr -I dts -O dtb -o blinkt.dtbo blinkt-overlay.dts
```

### Install the blinkt kernel module and device tree overlay
Install the kernel module and overlay file using the following commands:
```shell
sudo make modules_install
sudo depmod -a
sudo cp blinkt.dtbo /boot/overlays/
```
Now edit the file /boot/config.txt (or /boot/firmware/config.txt):
```shell
sudo nano /boot/config.txt
```
Add the following line at the end of the file:
```shell
dtoverlay=blinkt
```
Save and exit the editor.
Reboot your Raspberry Pi


## How to use

### Loading the kernel module
After installing and rebooting, open a command terminal and load the kernel module with the following command:
```shell
dan@raspberrypi:# sudo modprobe blinkt
```
The kernel module must always be loaded before a LED can be used. To automatically load the kernel module, so you don't have to load it manually after each reboot, open a command terminal and enter the following commands:
```shell
dan@raspberrypi:# sudo -s
root@raspberrypi:# echo "blinkt" >/etc/modules-load.d/blinkt.conf
root@raspberrypi:# exit
```


### Controlling the LEDs from a command shell
Now you should see the leds in the directory /sys/class/leds/:
```shell
dan@raspberrypi:# ls /sys/class/leds/
ACT      blinkt1  blinkt3  blinkt5  blinkt7     input3::capslock  input3::scrolllock  PWR
blinkt0  blinkt2  blinkt4  blinkt6  default-on  input3::numlock   mmc0
```
There are 8 leds, named blinkt0 ... blinkt7.
Now go to the first led, blinkt0:
```shell
dan@raspberrypi:# cd /sys/class/leds/blinkt0/
dan@raspberrypi:# ls
brightness  device  max_brightness  multi_index  multi_intensity  power  subsystem  trigger  uevent
```
To light up a LED we need to set both brightness and color intensity. The brightness is controlled by the file `` `brightness` ``. A value of 0 turns the LED off, and after loading the module, the brightness and color intensity values are by default set to 0. The maximum value can be found by looking at the file `` `max_brightness` ``:
```shell
dan@raspberrypi:# cat max_brightness 
255
```
Now set the brightness to some value between 0 and 255, for example 50. But first we need to have superuser access in order to modify the values.
```shell
dan@raspberrypi:# sudo -s 
```
Now, set the brightness to 50:
```shell
root@raspberrypi:# echo "50" >brightness 
```
Nothing happens! That's because we also need to set the intensity value of the colors we want. To see the available colors, look at the file `` `multi_index` ``:
```shell
root@raspberrypi:# cat multi_index
red green blue
```
And the values for red, green, and blue are set in the file `` `multi_index` ``:
```shell
root@raspberrypi:# cat multi_intensity
0 0 0
```
Now, finally, we can light the LED. Let us set it to a low green intensity:
```shell
root@raspberrypi:# echo "0 10 0" >multi_intensity
```

### Set a LED trigger
An example of how to set led `` `blinkt7` `` to a soft purple heartbeat blink:
```shell
dan@raspberrypi:# sudo -s
root@raspberrypi:# cd /sys/class/leds/blinkt7
root@raspberrypi:# echo "255" >brightness
root@raspberrypi:# echo "3 0 3" >multi_intensity
root@raspberrypi:# echo "heartbeat" >trigger
root@raspberrypi:# exit
```
The LED should now show a red heartbeat.


### Initialize LEDs with module parameters
When loading the kernel module, there are parameters that can be set to change the default values of all the LEDs.
To boot the Raspberry Pi with a soft read heartbeat, set module parameters in the file `` `/etc/modprobe.d/blinkt.conf` ``:
```shell
dan@raspberrypi:# sudo -s
root@raspberrypi:# cd /etc/modprobe.d
root@raspberrypi:# echo "options blinkt brgb0=255,3,0,0 trigger0=heartbeat" >blinkt.conf
root@raspberrypi:# exit
```
When the Raspberry Pi boots, a red heartbeat starts blinking.
