# hardware-raspberrypi-pico
pysigrok hardware plugin for using a RP2040 as a logic probe

## Quick start

To install the raspberry pi pico support and sigrok decoders do:

```sh
pip install pysigrok-hardware-raspberrypi-pico
```

Once installed you can list supported hardware, formats and decoders with:

```sh
$ pysigrok-cli -L

Supported hardware drivers:
  raspberrypi-pico      RaspberryPI PICO

Supported input formats:
  srzip srzip session file format data

Supported output formats:
  bits  ASCII rendering with 0/1
  srzip srzip session file format data

Supported transform modules:

Supported protocol decoders:
...
```

You'll need to install the sigrok-pico code onto your RP2040. The [source is on Github](https://github.com/pysigrok/sigrok-pico) and the [uf2 to install is as well](https://github.com/tannewt/sigrok-pico/blob/main/pico_sdk_sigrok/build/).

To find the CDC device for the RP2040:

```sh
$ pysigrok-cli --list-serial

Available serial ports:
  /dev/ttyUSB0 - CP2102N USB to UART Bridge Controller
  /dev/ttyACM2 - Pico - Board CDC
  /dev/ttyACM1 - nRF52 Connectivity
  /dev/ttyACM0 - Feather M0 Express - CircuitPython CDC control
```

To capture to a sigrok `.sr` file:

```sh
pysigrok-cli -d raspberrypi-pico:conn=/dev/ttyACM2 -C D16,D17,D18,D19 --samples 10 -c samplerate=10000000 -o test.sr
```

Open `test.sr` in PulseView.
