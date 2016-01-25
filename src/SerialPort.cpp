/*
  Serial port commnucation  Class.
  Copyright (C)  all right reserved. 2011 Isao Hara, AIST,Japan

*/
#include "stdafx.h"
#include "SerialPort.h"
#include <iostream>

#define DEV_INTERSIL
//#define DEBUG_CODE
//#define MACRO_MRAA
//#define DELAY_DE_RE
#define BAUD115
#ifdef BAUD115
#define DEFAULT_BAUD B115200
#else
#define DEFAULT_BAUD B460800
#endif
/**

*/
SerialCom::SerialCom():device(NULL),handle(H_NULL),mode(0),baudrate(DEFAULT_BAUD)
{
}
/**

*/
SerialCom::SerialCom(char *devname):handle(H_NULL),mode(0),baudrate(DEFAULT_BAUD)
{
  setDevPort(devname);
}

/**
  Deconstructor
*/
SerialCom::~SerialCom()
{
  closePort();
  delete device;
  handle = H_NULL;
}

/**
  set device name
*/
void SerialCom::setDevPort(char *devname){
  device = (char *)StrDup((const char *)devname);
}

/**
  open serial port
    byte size : 8
    parity    :  None
    stop bits : 1
    baudrate    :  115200 (default)
*/
int 
SerialCom::openPort()
{
#ifdef WIN32
  DCB dcb;

  if(mode == 1){
	  handle = CreateFile(device,
      GENERIC_READ | GENERIC_WRITE, 0, NULL,
      OPEN_EXISTING, 
      FILE_FLAG_OVERLAPPED,
      NULL);
  }else{
	  handle = CreateFile(device,
      GENERIC_READ | GENERIC_WRITE, 0, NULL,
      OPEN_EXISTING, 
      FILE_ATTRIBUTE_NORMAL,
      NULL);
  }

  if (handle == INVALID_HANDLE_VALUE){
    char dev[32];
	sprintf(dev, "\\\\.\\%s",device);

	//std::cerr << "Try to open: " << dev << std::endl;
    if(mode == 1){
	  handle = CreateFile(dev,
      GENERIC_READ | GENERIC_WRITE, 0, NULL,
      OPEN_EXISTING, 
      FILE_FLAG_OVERLAPPED,
      NULL);
    }else{
	  handle = CreateFile(dev,
      GENERIC_READ | GENERIC_WRITE, 0, NULL,
      OPEN_EXISTING, 
      FILE_ATTRIBUTE_NORMAL,
      NULL);
    }

  }

  if (handle != INVALID_HANDLE_VALUE){
  GetCommState(handle, &dcb);

  dcb.BaudRate = baudrate;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.fParity = FALSE;
  dcb.StopBits = ONESTOPBIT;
  SetCommState(handle, &dcb);
  }else{
     handle = H_NULL;
   return -1;
  }
  return (int)handle;
#else

#ifdef MACRO_MRAA // @@@
  char* board_name = mraa_get_platform_name();
//  fprintf(stdout, "hello mraa\n Version: %s\n Running on %s\n", mraa_get_version(), board_name);

//  gpio_25 = mraa_gpio_init(25);
//  gpio_26 = mraa_gpio_init(26);
//  gpio_35 = mraa_gpio_init(35);
//  mraa_gpio_use_mmaped(gpio_20, 1);
//  mraa_gpio_dir(gpio_25, MRAA_GPIO_OUT);
//  mraa_gpio_dir(gpio_26, MRAA_GPIO_IN);
//  mraa_gpio_dir(gpio_35, MRAA_GPIO_OUT);

//  uart_01 = mraa_uart_init(0);
  uart_01 = mraa_uart_init_raw(device);
  if (uart_01 == NULL) {
    fprintf(stderr, "UART failed to setup\n");
    uart_01 = H_NULL;
    return -1;
  }
  printf("Device path: %s\n", mraa_uart_get_dev_path(uart_01));
  mraa_uart_set_baudrate(uart_01, DEFAULT_BAUD);
  mraa_uart_set_mode(uart_01, 8, MRAA_UART_PARITY_NONE, 1);
  mraa_uart_set_flowcontrol(uart_01, 0, 0);
  mraa_uart_set_timeout(uart_01, 0, 0, 0);
  handle = 4;
  return (int)1;
#else // MACRO_MRAA
  struct termios tio;

  if(mode == 1){
    handle = open(device, O_RDWR|O_NONBLOCK);
  }else{
    handle = open(device, O_RDWR);
  }
  if(handle < 0){
    handle = H_NULL;
    return -1;
  }
  printf("handle: %d / mode:%d\n", handle, mode);

  // @@@
  mraa_init();
  gpio_20 = mraa_gpio_init(20); // GPIO12/J18-7
  mraa_gpio_use_mmaped(gpio_20, 1); // FastIO
  mraa_gpio_dir(gpio_20, MRAA_GPIO_OUT);

  memset(&tio, 0, sizeof(tio));
//  tio.c_iflag = IXON|IXOFF;
  tio.c_cflag = CS8|CREAD|CLOCAL;
//  tio.c_cflag = CS8|CREAD|CLOCAL|CRTSCTS;
  tio.c_cc[VMIN] = 1;
  cfsetospeed(&tio, baudrate);
  cfsetispeed(&tio, baudrate);
  tcsetattr(handle, TCSANOW, &tio);
  return (int)handle;
#endif // MACRO_MRAA

#endif

}

/*
 *  Change baudrate
 */
int
SerialCom::setBaudrate(int b)
{
  baudrate = b;
  if(handle != H_NULL){
#ifdef WIN32
    DCB dcb;

	GetCommState(handle, &dcb); 
	dcb.BaudRate = baudrate;
	SetCommState(handle, &dcb);
#else

#ifdef MACRO_MRAA // @@@
  mraa_uart_set_baudrate(uart_01, DEFAULT_BAUD);
#else // MACRO_MRAA
    struct termios tio;
    tcgetattr(handle, &tio);
    cfsetospeed(&tio, b);
    cfsetispeed(&tio, b);
    tcsetattr(handle, TCSANOW, &tio);
#endif // MACRO_MRAA

#endif
  }
  return b;
}

/**
  close serial port
*/
void SerialCom::closePort(){
  if(this->handle != H_NULL){
#ifdef WIN32
    CloseHandle(this->handle);
#else

#ifdef MACRO_MRAA // @@@
  mraa_uart_stop(uart_01);
  mraa_deinit();
#else // MACRO_MRAA
  close(this->handle);
  // @@@
  mraa_gpio_dir(gpio_20, MRAA_GPIO_IN);
  mraa_deinit();
#endif // MACRO_MRAA

#endif
    this->handle = H_NULL;
  }
}

/**
  recieve data from serial port (low level)
 */
int SerialCom::Read(char *data, int len){
#ifdef WIN32
  DWORD dwRead;

  if(mode == 1){
    OVERLAPPED lpo;
    ZeroMemory(&lpo, sizeof(lpo));
    lpo.hEvent = NULL;
    ReadFile(handle, data, len, &dwRead, &lpo);
    if(GetLastError() != ERROR_IO_PENDING){
      return -1;
    }
  }else{
    if(ReadFile(handle, data, len, &dwRead, NULL) == FALSE){
      return -1;
    }
  }
  return dwRead;
#else

#ifdef MACRO_MRAA // @@@
  return mraa_uart_read(uart_01, data, len);
#else // MACRO_MRAA
  return read(handle, data, len);
#endif // MACRO_MRAA

#endif
}

/**
 * recieve data from serial port. this function is blocked until all date recieved. (high level)
 */
int SerialCom::recieveData(char *data, int len){
  int i,c, recieved;

  recieved = 0;

  /*  Waiting for a moment  */
  for(i = 0; i <100; i++){ 
    if(this->chkBuffer() >= len){
      break;
    }
    Sleep(1);
  }

  if(i >= 100) {
    this->clearBuffer();
    return -1;
  }
  do{
    c = Read(data+recieved, len-recieved);
    if(c < 0){
    std::cerr << "ERROR in read" << std::endl;
      return -1;
    }
  recieved += c;
  } while(recieved < len);

  return len;
}

/**
 send data to serial port (low level)
*/
int SerialCom::Write(char *data, int len){
#ifdef WIN32
  DWORD dwWrite;
  if(mode == 1){
    OVERLAPPED lpo;
    ZeroMemory(&lpo, sizeof(lpo));
    lpo.hEvent = NULL;
    WriteFile(handle, data, len, &dwWrite, &lpo);
    if(GetLastError() != ERROR_IO_PENDING){
      return -1;
    }
  }else{
    if(WriteFile(handle, data, len, &dwWrite, NULL) == FALSE){
    return -1;
    }
  }
  return dwWrite;
#else

#ifdef MACRO_MRAA // @@@
  int res;
  res = mraa_uart_write(uart_01, data, len);
  mraa_uart_flush(uart_01);
#else // MACRO_MRAA
  int res;
  int n=99;
  res = write(handle, data, len);
#ifdef DELAY_DE_RE
  do { ioctl(this->handle, TIOCOUTQ, &n); } while (n);
#endif
#endif // MACRO_MRAA
  return res;

#endif
}

/**
 *  send date to serial port. this function block until all data sent.
 */
int SerialCom::sendData(char *data, int len){
  int c, sent=0;

  this->clearBuffer(); // @@@
  mraa_gpio_write(gpio_20, 1); // @@@
  do{
    c = Write(data+sent, len-sent);
    if(c < 0){
    std::cerr << "ERROR in SerialCom::sendData" << std::endl;
    return -1;
    }
    sent += c;
  }while(sent < len);
#ifdef DELAY_DE_RE
#ifdef BAUD115
  usleep(570); // @@@115200
#else
  usleep(40); // @@@460800
#endif
#endif
  mraa_gpio_write(gpio_20, 0); // @@@
#ifdef DEV_INTERSIL
  if (sent) { Read(data, sent); }
#endif
  return sent;
}

/**
  check buffer and return size of data.
*/
int SerialCom::chkBuffer(){
  int n;

#ifdef WIN32
  DWORD lpErrors;
  COMSTAT lpStat;
  ClearCommError(this->handle, &lpErrors, &lpStat);

  n = lpStat.cbInQue;
#else
#ifndef TIOCINQ
#define TIOCINQ FIONREAD
#endif
  // @@@
  mraa_gpio_write(gpio_20, 0);
  ioctl(this->handle, TIOCINQ, &n);
#endif

  return n;
}

/*
 *  clear date in the buufer of commnuication port.
 */
int SerialCom::clearBuffer(){
  int i;
  char c;
  int n = this->chkBuffer();

  for(i=0;i<n;i++)
  {
    if(Read(&c, 1) != 0){
#if 0 // @@@ UART->RS485 tuned
      std::cerr << "ERROR!!: fail to read byte." << std::endl;
      return -1;
#endif
    }
  }
  return n;
}

/**
   print packets for debugging
*/
void SerialCom::printPacket(char *data, int len){
  int i;
  for(i=0; i<len; i++){
    fprintf(stderr, " %02x",(unsigned char)data[i]);
  }
  fprintf(stderr, "\n");
  return;
}

/*
 *  return braurate.
 */
int SerialCom::getBaudrate(){
  return baudrate;
}

/*
 *  return current commnication mode.
 */
int SerialCom::getMode(){
  return mode;
}

/*
 *  set commnication mode. if the commication port is already opened, this function will fail.
 */
int SerialCom::setMode(int mode){
  if(handle != H_NULL){
    std::cerr << "Fail to chandge mode" << std::endl;
  }else{
    mode = mode;
  }
  return mode;
}

int SerialCom::isConnected(){
  if(handle == H_NULL){
    return -1;
  }else{
    return 1;
  }
}
