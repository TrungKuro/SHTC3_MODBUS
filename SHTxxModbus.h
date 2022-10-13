/*!
 *  @file SHTxxModbus.h
 *
 *  This is a library for the ...
 *
 *  These sensors use RS485 to communicate, 2 pins are required to interface.
 *
 *  TrungKuro (Hshop).
 */

#ifndef SHT_MODBUS_H
#define SHT_MODBUS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

/* ------------------------------------------------------------------------- */

// #define ENABLE_DEBUG

#if defined(ENABLE_DEBUG)
#define Debug Serial
#define DEBUG_PRINT(...) Debug.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Debug.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

/* ------------------------------------------------------------------------- */

// #define CMD_READ 0x03  // Command Read
// #define CMD_WRITE 0x06 // Command Write

// #define REG_HUMI 0x0000 // Temperature (Only-Read)
// #define REG_TEMP 0x0001 // Humidity (Only-Read)

// #define REG_CODE_ADDR 0x07D0 // Default: Address 1 (0x01)
// #define REG_CODE_BAUD 0x07D1 // Default: Baudrate 4800 bit/s (0x01)

#define BAUD_2400 0x00
#define BAUD_4800 0x01
#define BAUD_9600 0x02
#define INVALID_BAUD 0xFF

#define MAX_SHT_TIMEOUT (20000) // Unit (ms)
#define MIN_SHT_TIMEOUT (100)   // Unit (ms)

/* ------------------------------------------------------------------------- */

enum
{
  HARD_SERIAL,
  SOFT_SERIAL
};

typedef struct dataSHT
{
  float temperatureC;
  float temperatureF;
  float humidity;
};

/* ------------------------------------------------------------------------- */

class SHT
{
private:
  /* - [Addr(1)] - [Cmd(1)] - [Reg(2)] - [Data(2)] - [L_Crc(1)] - [H_Crc(1)] - */

  uint8_t getValue[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00}; // 0x xx 03 00 00 00 02 xx xx

  uint8_t setBaud[8] = {0x00, 0x06, 0x07, 0xD1, 0x00, 0x00, 0x00, 0x00}; // 0x xx 06 07 D1 xx xx xx xx
  uint8_t getBaud[8] = {0x00, 0x03, 0x07, 0xD1, 0x00, 0x01, 0x00, 0x00}; // 0x xx 03 07 D1 00 01 xx xx

  uint8_t setAddr[8] = {0x00, 0x06, 0x07, 0xD0, 0x00, 0x00, 0x00, 0x00}; // 0x xx 06 07 D0 xx xx xx xx
  uint8_t getAddr[8] = {0xFF, 0x03, 0x07, 0xD0, 0x00, 0x01, 0x91, 0x59}; // 0x FF 03 07 D0 00 01 91 59

  Stream *port;
  uint8_t typeSerial;

  uint8_t _baud;
  uint8_t _addr;
  uint16_t _timeOut = MIN_SHT_TIMEOUT;

public:
  SHT(uint8_t baud = BAUD_4800, uint8_t addr = 0x01);                                       // Constructor - HardwareSerial (RX=0) (TX=1)
  SHT(uint8_t rxPin = 2, uint8_t txPin = 3, uint8_t baud = BAUD_4800, uint8_t addr = 0x01); // Constructor - SoftwareSerial (RX=2) (TX=3)
  virtual ~SHT() { delete port; }                                                           // Virtual Destructor

  void begin(uint16_t baud);
  void setTimeout(uint16_t timeOut);

  dataSHT getData();

  float readTemperature(bool isDegreeCelsius = true);
  float readHumidity();

  uint16_t readBaudrate();
  uint8_t readAddress();

  /* Return "TRUE" when success */
  bool setBaudrate(uint8_t baud);
  bool setAddress(uint8_t addr);
};

/* ------------------------------------------------------------------------- */

#endif
