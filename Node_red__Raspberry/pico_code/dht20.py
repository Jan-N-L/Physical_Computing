from machine import I2C
from time import sleep_ms

"""
DHT20 temperature & humidity sensor (I2C, address 0x38)

Usage:
    sensor = DHT20(i2c)
    temp, humid = sensor.measure()   # one measurement, both values
"""

class DHT20(object):
    def __init__(self, i2c):
        self.i2c = i2c
        if (self.read_status() & 0x80) == 0x80:
            self.init()

    def measure(self):
        """
        Returns:
            tuple: (temperature in °C, relative humidity in %)
        """
        data = self._read_raw()
        return self._calc_temperature(data), self._calc_humidity(data)

    def temperature(self):
        return self.measure()[0]

    def humidity(self):
        return self.measure()[1]

    # ---- Helper functions ----

    def _read_raw(self):
        self.i2c.writeto(0x38, bytes([0xac, 0x33, 0x00]))
        sleep_ms(80)
        cnt = 0
        while (self.read_status() & 0x80) == 0x80:   # sensor busy
            sleep_ms(1)
            cnt += 1
            if cnt >= 100:
                break
        return list(self.i2c.readfrom(0x38, 7, True))

    def _calc_temperature(self, data):
        temp = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0xfffff
        return (temp * 200 * 10 / 1024 / 1024 - 500) / 10

    def _calc_humidity(self, data):
        hum = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        return (hum * 100 * 10 / 1024 / 1024) / 10

    def read_status(self):
        data = self.i2c.readfrom(0x38, 1, True)
        return data[0]

    def init(self):
        self.i2c.writeto(0x38, bytes([0xa8, 0x00, 0x00]))
        sleep_ms(10)
        self.i2c.writeto(0x38, bytes([0xbe, 0x08, 0x00]))

    def calc_crc8(self, data):
        crc = 0xff
        for i in data[:-1]:
            crc ^= i
            for j in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = (crc << 1)
        return crc
