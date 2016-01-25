/*
   Sample program to control G-001
  Copyright(C) 2011, Isao HARA, AIST,Japan

*/
#ifndef __C_UTILS_H__
#define __C_UTILS_H__

//#include "stdafx.h"

#ifdef __cplusplus
extern "C" {
#endif

// @@@
#define DEV_INTERSIL
//#define DEBUG_PRINT
//#define EXEC_CONSOLE
//#define MACRO_MRAA
//#define 	CTL_DE_RE
//#define DELAY_DE_RE
#define BAUD115
#ifdef BAUD115
#define DEFAULT_BAUD B115200
#else
#define DEFAULT_BAUD B460800
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>

#ifdef WIN32
#include <sys/types.h>
#include <sys/timeb.h>
#include <process.h>
#include <windows.h>

#ifdef GR001_EXPORTS
#define C_DECLSPEC	__declspec(dllexport)
#else
#define C_DECLSPEC  __declspec(dllimport)
#endif

#else
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <linux/joystick.h> // @@@

#define C_DECLSPEC
#endif


/**** Mutex ****/

#ifdef WIN32
#define DEV_STR      "COM%d"
#define SSCANF      sscanf_s
#define SPRINTF      sprintf_s
#define THREAD_FUNC    void
#define H_NULL      NULL
#define THREAD_HANDLE  HANDLE
#define MUTEX_HANDLE  HANDLE
#define StrDup(s)    _strdup(s)

void Pthread_Mutex_Init(HANDLE *hM, void *arg);
int Pthread_Create(HANDLE *hT, void *attr,void (*func)(void *), void *arg);
void Pthread_Detach(HANDLE hT);
void Pthread_Mutex_Destroy(HANDLE *hM);
void Pthread_Join(HANDLE hT, void *thread_return);
void Pthread_Exit(HANDLE hT);
#else
#define HANDLE      int
#define DEV_STR      "/dev/ttyMFD%d"
#define SSCANF      sscanf
#define SPRINTF      sprintf
#define Sleep(n)    usleep(n*1000)
#define THREAD_FUNC    void *
#define H_NULL      0
#define THREAD_HANDLE  pthread_t
#define MUTEX_HANDLE  pthread_mutex_t
#define StrDup(s)    strdup(s)

#define Pthread_Mutex_Init pthread_mutex_init
#define Pthread_Create  pthread_create
#define Pthread_Detach  pthread_detach
#define Pthread_Mutex_Destroy  pthread_mutex_destroy
#define Pthread_Join  pthread_join
#define Pthread_Exit(x)  pthread_exit((void *)x)

#endif


#define PI 3.14159265358979323846
#define Rad2Deg 180.0 / PI
#define Deg2Rad PI / 180.0

/*
 Functions
*/
C_DECLSPEC char calcSum(char *data, int len);
C_DECLSPEC short convC2S(unsigned char *data);
C_DECLSPEC char *getNumber(char *str, short *val);
C_DECLSPEC char *getDoubleNumber(char *str, double *val);
C_DECLSPEC int getCurrentTime();
C_DECLSPEC bool FileExists(const char* fname, const char *ext);

#ifdef __cplusplus
};
#endif

#endif
