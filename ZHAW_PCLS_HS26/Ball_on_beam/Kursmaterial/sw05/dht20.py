from machine import I2C
from time import sleep_ms

class DHT20(object):
    def __init__(self, i2c):
        self.i2c = i2c
        if (self.read_status() & 0x80) == 0x80:
            self.init()
            
    def measure(self):
        self.i2c.writeto(0x38, bytes([0xac,0x33,0x00]))
        sleep_ms(80)
        cnt = 0
        while (self.read_status() & 0x80) == 0x80:
            sleep_ms(1)
            if cnt >= 100:
                cnt += 1
                break
        data = self.i2c.readfrom(0x38, 7, True)
        n = []
        for i in data[:]:
            n.append(i)
        return n
        
    def read_status(self):
        data = self.i2c.readfrom(0x38, 1, True)
        return data[0]
    
    def init(self):
        self.i2c.writeto(0x38, bytes([0xa8,0x00,0x00]))
        sleep_ms(10)
        self.i2c.writeto(0x38, bytes([0xbe,0x08,0x00]))
        
    def calc_crc8(self,data):
        crc = 0xff
        for i in data[:-1]:
            crc ^= i
            for j in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = (crc << 1)
        return crc
    
    def temperature(self):
        data = self.measure()
        temp = 0
        if 1:
            temp = (temp | data[3]) << 8
            temp = (temp | data[4]) << 8
            temp = temp | data[5]
            temp = temp & 0xfffff
            temp = (temp * 200 * 10 / 1024 / 1024 - 500)/10
        return temp

    def humidity(self):
        data = self.measure()
        hum = 0
        if 1:
            hum = (hum | data[1]) << 8
            hum = (hum | data[2]) << 8
            hum = hum | data[3]
            hum = hum >> 4
            hum = (hum * 100 * 10 / 1024 / 1024)/10
        return hum
