#include "GT911.h"
#include <Wire.h>

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

// Interrupt handling
volatile bool gt911IRQ = false;

#if defined(ESP8266)
void ICACHE_RAM_ATTR _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#elif defined(ESP32)
void IRAM_ATTR _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#else
void _gt911_irq_handler() {
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#endif

GT911::GT911(TwoWire *twi) : _wire(twi ? twi : &Wire) {

}

void GT911::reset() {
  delay(1);

  pinMode(_intPin, OUTPUT);
  pinMode(_rstPin, OUTPUT);

  digitalWrite(_intPin, LOW);
  digitalWrite(_rstPin, LOW);

  delay(11);

  digitalWrite(_intPin, _addr == GT911_I2C_ADDR_28);

  delayMicroseconds(110);
  pinMode(_rstPin, INPUT);

  delay(6);
  digitalWrite(_intPin, LOW);

  delay(51);
}

void GT911::i2cStart(uint16_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg >> 8);
  _wire->write(reg & 0xFF);
}

bool GT911::write(uint16_t reg, uint8_t data) {
  i2cStart(reg);
  _wire->write(data);
  return _wire->endTransmission() == 0;
}

uint8_t GT911::read(uint16_t reg) {
  i2cStart(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)1);
  while (_wire->available()) {
    return _wire->read();
  }
  return 0;
}

bool GT911::writeBytes(uint16_t reg, uint8_t *data, uint16_t size) {
  i2cStart(reg);
  for (uint16_t i = 0; i < size; i++) {
    _wire->write(data[i]);
  }
  return _wire->endTransmission() == 0;
}

bool GT911::readBytes(uint16_t reg, uint8_t *data, uint16_t size) {
  i2cStart(reg);
  _wire->endTransmission();

  uint16_t index = 0;
  while (index < size) {
    uint8_t req = _min(size - index, I2C_BUFFER_LENGTH);
    _wire->requestFrom(_addr, req);
    while (_wire->available()) {
      data[index++] = _wire->read();
    }
    index++;
  }

  return size == index - 1;
}

uint8_t GT911::calcChecksum(uint8_t *buf, uint8_t len) {
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++) {
    ccsum += buf[i];
  }

  return (~ccsum) + 1;
}

uint8_t GT911::readChecksum() {
  return read(GT911_REG_CHECKSUM);
}

int8_t GT911::readTouches() {
  uint32_t timeout = millis() + 20;
  do {
    uint8_t flag = read(GT911_REG_COORD_ADDR);
    if ((flag & 0x80) && ((flag & 0x0F) < GT911_MAX_CONTACTS)) {
      write(GT911_REG_COORD_ADDR, 0);
      return flag & 0x0F;
    }
    delay(1);
  } while (millis() < timeout);

  return 0;
}

bool GT911::readTouchPoints() {
  bool result = readBytes(GT911_REG_COORD_ADDR + 1, (uint8_t*)_points, sizeof(GTPoint) * GT911_MAX_CONTACTS);
  /*
  if (result && _rotation != Rotate::_0) {
    for (uint8_t i = 0; i < 5; i++) {
        _points[i].x = _points[i].x;
        _points[i].y = _points[i].y;
    }
  }
  */
  if (result && _rotation != Rotate::_0) {
    for (uint8_t i = 0; i < 5; i++) {
	  if (_rotation == Rotate::_90) {
        int temp = _points[i].x;
		_points[i].x = _info.yResolution - _points[i].y;
        _points[i].y = temp;
      }
	  if (_rotation == Rotate::_270) {
        int temp = _points[i].x;
		_points[i].x = _info.yResolution - _points[i].y;
        _points[i].y = temp;
      }
	  if (_rotation == Rotate::_180) {
        int temp = _points[i].x;
		_points[i].x = _info.yResolution - _points[i].y;
        _points[i].y = temp;
      }
    }
  }

  return result;
}

bool GT911::begin(int8_t intPin, int8_t rstPin, uint8_t addr, uint32_t clk) {
  _intPin = intPin;
  _rstPin = rstPin;
  _addr = addr;

  if (_rstPin > 0) {
    delay(300);
    reset();
    delay(200);
  }

  if (intPin > 0) {
    pinMode(_intPin, INPUT);
    attachInterrupt(_intPin, _gt911_irq_handler, FALLING);
  }

  _wire->begin(7, 8, 400000);
  _wire->setClock(clk);
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() == 0) {
    readInfo(); // Need to get resolution to use rotation
    return true;
  }
  return false;
}

void GT911::fwResolution(uint16_t maxX, uint16_t maxY) {
	//Serial.begin(115200);
	//Serial0.begin(115200);
	//delay(1000);
	//Serial.println("ready");
	//Serial0.println("ready0");
	if(_info.yResolution == maxY && _info.xResolution == maxX) return;
	uint8_t len1 = GOODIX_REG_CONFIG_MIDDLE - GOODIX_REG_CONFIG_DATA +1;
	uint8_t len2 = GOODIX_REG_CONFIG_END - GOODIX_REG_CONFIG_MIDDLE;
	uint8_t buf1[len1];
	uint8_t buf2[len2];
	uint8_t buf3[2];
	uint8_t buf[len1+len2];
	readBytes(GOODIX_REG_CONFIG_DATA, buf1, len1);
    readBytes(GOODIX_REG_CONFIG_MIDDLE+1, buf2, len2);
    memcpy(buf, buf1, sizeof(buf1));
	memcpy(buf+sizeof(buf1), buf2, sizeof(buf2));

  	buf[0]++;
	buf[1] = (maxX & 0xff);
	buf[2] = (maxX >> 8);
	buf[3] = (maxY & 0xff);
	buf[4] = (maxY >> 8);
	buf3[0] = calcChecksum(buf, len1+len2);
    buf3[1] = 0x01;

	writeBytes(GOODIX_REG_CONFIG_DATA, buf, len1+len2);
    writeBytes(GOODIX_REG_CONFIG_END+1, buf3, 2);
	//for(int n = 0; n < len1+len2; n++){
	//  Serial.println(buf[n]);
	//  Serial0.println(buf[n]);
	//}
	//memcpy(touchBUF, buf, len1+len2);
	//touchBUFlen = len1+len2;
}

bool GT911::productID(uint8_t *buf, uint8_t len) {
  if (len < 4) {
    return false;
  }

  memset(buf, 0, 4);
  return readBytes(GT911_REG_ID, buf, 4);
}

GTConfig* GT911::readConfig() {
  readBytes(GT911_REG_CFG, (uint8_t*)&_config, sizeof(_config));

  if (readChecksum() == calcChecksum((uint8_t*)&_config, sizeof(_config))) {
    _configLoaded = true;
    return &_config;
  }
  return nullptr;
}

bool GT911::writeConfig() {
  uint8_t checksum = calcChecksum((uint8_t*)&_config, sizeof(_config));
  if (_configLoaded && readChecksum() != checksum) { // Config is different
    writeBytes(GT911_REG_CFG, (uint8_t*)&_config, sizeof(_config));

    uint8_t buf[2] = { checksum, 1 };
    writeBytes(GT911_REG_CHECKSUM, buf, sizeof(buf));
    return true;
  }
  return false;
}

GTInfo* GT911::readInfo() {
  readBytes(GT911_REG_DATA, (uint8_t*)&_info, sizeof(_info));
  return &_info;
}

uint8_t GT911::touched(uint8_t mode) {
  bool irq = false;
  if (mode == GT911_MODE_INTERRUPT) {
    noInterrupts();
    irq = gt911IRQ;
    gt911IRQ = false;
    interrupts();
  } else if (mode == GT911_MODE_POLLING) {
    irq = true;
  }

  uint8_t contacts = 0;
  if (irq) {
    contacts = readTouches();

    if (contacts > 0) {
      readTouchPoints();
    }
  }

  return contacts;
}

GTPoint GT911::getPoint(uint8_t num) {
  return _points[num];
}

GTPoint *GT911::getPoints() {
  return _points;
}

void GT911::SetRotation(int rt) {
  if(rt == 0) _rotation = Rotate::_90;
  if(rt == 1) _rotation = Rotate::_180;
  if(rt == 2) _rotation = Rotate::_270;
  if(rt == 3) _rotation = Rotate::_0;
}
void GT911::setRotation(Rotate rotation) {
  _rotation = rotation;
}
