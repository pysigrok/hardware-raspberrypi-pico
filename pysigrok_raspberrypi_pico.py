"""PySigrok driver for rp2040 logic capture"""

__version__ = "0.0.1"

import serial

from sigrokdecode import SR_KHZ, SR_MHZ

class PicoDriver:
    name = "raspberrypi-pico"
    longname = "RaspberryPI PICO"

    samplerates = [
        SR_KHZ(5),
        SR_KHZ(6),
        SR_KHZ(8),
        SR_KHZ(10),
        SR_KHZ(20),
        SR_KHZ(30),
        SR_KHZ(40),
        SR_KHZ(50),
        SR_KHZ(60),
        SR_KHZ(80),
        SR_KHZ(100),
        SR_KHZ(125),
        SR_KHZ(150),
        SR_KHZ(160), # max rate of 3 ADC channels that has integer divisor/dividend
        SR_KHZ(200),
        SR_KHZ(250), # max rate of 2 ADC channels
        SR_KHZ(300),
        SR_KHZ(400),
        SR_KHZ(500),
        SR_KHZ(600),
        SR_KHZ(800),
        # Give finer granularity near the thresholds of RLE effectiveness ~1-4Msps
        # Also use 1.2 and 2.4 as likely max values for ADC overclocking
        SR_MHZ(1),
        SR_MHZ(1.2),
        SR_MHZ(1.5),
        SR_MHZ(2),
        SR_MHZ(2.4),
        SR_MHZ(3),
        SR_MHZ(4),
        SR_MHZ(5),
        SR_MHZ(6),
        SR_MHZ(8),
        SR_MHZ(10),
        SR_MHZ(15),
        SR_MHZ(20),
        SR_MHZ(30),
        SR_MHZ(40),
        SR_MHZ(60),
        # The baseline 120Mhz PICO clock won't support an 80 or 100
        # with non fractional divisor, but an overclocked version or one
        # that modified sysclk could
        SR_MHZ(80),
        SR_MHZ(100),
        SR_MHZ(120),
        # These may not be practically useful, but someone might want to
        # try to make it work with overclocking
        SR_MHZ(150),
        SR_MHZ(200),
        SR_MHZ(240)
    ]

    def __init__(self, channellist="", conn=""):
        self.serial = serial.Serial(conn)
        self._samplerate = 5000
        self.data = []

        self.serial.reset_input_buffer()
        self.serial.write(b"*")
        self.serial.reset_input_buffer()

        print("requesting id")
        self.serial.write(b"i\n")
        info = self.serial.read(17)
        print(info)
        print(len(info))
        if not info.startswith(b"SRPICO,A"):
            raise RuntimeError()
        _, channelinfo, self.version = info.split(b",")
        self.version = int(self.version)

        self.num_a_channels = int(channelinfo[1:3])
        self.analog_bytes = int(channelinfo[3:4])
        self.num_d_channels = int(channelinfo[-2:])
        print(self.num_a_channels, self.analog_bytes, self.num_d_channels)

        self.analog_channels = [f"A{n:d}" for n in range(self.num_a_channels)]
        self.logic_channels = [f"D{n+2:d}" for n in range(self.num_d_channels)]

        self.enabled_channels = ["D16", "D17"]

    def __del__(self):
        self.serial.close()

    @property
    def samplerate(self):
        return self._samplerate

    @samplerate.setter
    def samplerate(self, value):
        if value not in self.samplerates:
            raise ValueError()
        self._samplerate = value

    def send_w_ack(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        self.serial.write(data)
        response = self.serial.read(1)
        if response != b"*":
            raise RuntimeError()

    def acquire(self, sample_count):

        digital_enables = ["D"] * self.num_d_channels
        for channel in self.analog_channels:
            i = self.analog_channels.index(channel)
            enabled = 1 if channel in self.enabled_channels else 0
            self.send_w_ack(f"A{enabled:d}{i:d}\n")
        for channel in self.logic_channels:
            i = self.logic_channels.index(channel)
            enabled = 1 if channel in self.enabled_channels else 0
            self.send_w_ack(f"D{enabled:d}{i:d}\n")
            if enabled:
                digital_enables[i] = "E"
        digital_enables = "".join(digital_enables)
        digital_enables = digital_enables.strip("D")
        if "D" in digital_enables:
            raise RuntimeError("Digital pins must be continuous")

        self.send_w_ack(f"L{sample_count:d}\n")

        self.serial.write(f"R{self.samplerate:d}\n".encode("utf-8"))
        response = self.serial.read(1)
        if response == b"*":
            warning = self.serial.read(self.serial.in_waiting)
            if warning:
                print(warning)
        else:
            raise RuntimeError()

        self.serial.write(b"F\n")

        samples_read = 0
        while samples_read < sample_count:
            while self.serial.in_waiting == 0:
                pass
            data = self.serial.read(self.serial.in_waiting)
            print(data)
            samples_read += len(data)

