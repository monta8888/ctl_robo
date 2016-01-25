/*
  Serial Port Communication Class
  Copyright(C) all right reserved 2011 Isao Hara,AIST,Japan
*/

#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#include <mraa.h> // @@@
#include "CUtils.h"

/*
 * Baudrate 
 */
#ifdef WIN32
#define B0  0
#define B2400  2400
#define B4800  4800
#define B9600  9600
#define B19200  19200
#define B38400  38400
#define B7200  7200
#define B14400  14400
#define B28800  28800
#define B57600  57600
#define B76800  76800
#define B115200  115200
#define B230400  230400
#endif

/*
 *  Serial Port Class
 */
class SerialCom
{
public:
  /*
   * Constrauctor
   */
  SerialCom();
  SerialCom(char *devname);
  /*
   * Deconstructor
   */
  ~SerialCom();

  /*
   *  set device name.
   */
  void setDevPort(char*devname);
  /*
   * open commnication port. if it is sucess, this function return positive value.
   * if  it is failed, return nagative value (-1).
   */
  int openPort();
  /*
   *  close commnication port.
   */
  void closePort();

  /*
   * Low level communication
   */
  /*
   * Read : this function is compatible with 'read' function in POSIX-C
   */
  int Read(char *data, int len);
  /*
   * Write: this function is compatible with 'write' function in POSIX-C
   */
  int Write(char *data, int len);

  /*
   * High level commnucation
   */
  /*
   * This function is blocked until all data will be recieved.
   */
  int recieveData(char *data, int len);
  /*
   * This function is blcoked until all data will be sent.
   */
  int sendData(char *data, int len);
  /*
   * Check a buffer and returns size of data.
   */
  int chkBuffer();
  /*
   * Clear a buffer
   */
  int clearBuffer();
  /*
   *  print packet data for debugging commnication packets
   */
  void printPacket(char *data, int len);
  /*
   *  return current baudrate.
   */
  int getBaudrate();
  /*
   *  change baurate.
   */
  int setBaudrate(int baud_rate);
  /*
   * return commnication mode: 1 -> non blocking mode, other blocking mode.
   */
  int getMode();
  /*
   * set commnication mode. if the commnication port already opend, this function fail to set mode.
   */
  int setMode(int mode);

  int isConnected();

public:
  char *device;  // device name. e.g. /dev/ttyS0 for Unix, COM1 for Windows
  int baudrate;   // Baudrate 
  int mode;      // Communication mode. mode=1 -> Non Blocking mode, other Blocking mode

  HANDLE handle; // Handle for serial communication
  mraa_uart_context uart_01; // @@@
#ifdef CTL_DE_RE // @@@
  mraa_gpio_context gpio_20; // @@@
#endif // CTL_DE_RE
  mraa_gpio_context gpio_25; // @@@
  mraa_gpio_context gpio_26; // @@@
  mraa_gpio_context gpio_35; // @@@
  int send_len;
};

#endif

