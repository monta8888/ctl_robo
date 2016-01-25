#include <iostream>
using namespace std;

 #ifdef WIN32
 #include "stdafx.h"
 #endif

 #include "GROBO.h"

 static int loop = 1;
 static GR001 *G_ROBO;

 /*
  Signal handler for Ctr-C
  Close com_fd and stop threads
 */
 void
 sighandler(int x)
 {
   loop = 0;
   G_ROBO->setServo(0, 1);
   delete G_ROBO;
   return;
 }

 /*
  *  M A I N
  */
 int
 main(int argc, char* argv[])
 {
   /*   �ϐ��̒�`�@*/
   char line[1024];

 #ifdef WIN32
    char *devname = "COM1";
 #else
// @@@    char *devname = "/dev/ttyMFD1";
    char *devname = "/dev/ttyUSB0";
 #endif

   /* Ctrl+C ���������Ƃ��̏�����o�^ */
   signal(SIGINT, sighandler);

   /* �R�}���h�̈����ŃV���A���|�[�g���w�肷�� */
   if(argc > 1){
     devname  = (char *)argv[1];
   }

   /* GR001�̃N���X�̏������ƃ��{�b�g�ւ̐ڑ��@*/
   G_ROBO = new GR001((char *)devname);
   if( G_ROBO->connect() < 0 ){
     std::cerr << "Error: can't find G-Robot: " <<  devname << std::endl;
   }
   if(G_ROBO->startThread() == 0){
     std::cerr << "Error: fail to create thread" << std::endl;
     delete G_ROBO;
     exit(1);
   }

   /* ����t�@�C���̃f�B���N�g�����w�肷��@*/
   G_ROBO->setMotionDir("motion");

   /* �R�}���h���̓��[�v�@*/
   while(loop){
     /* �R�}���h���̓v�����v�g���o���āA��s�̓��͂�҂� */
     std::cout << "G-001 >";
     scanf("%s",line);

    /* ���͂��ꂽ�R�}���h������ɉ����āA���{�b�g�𓮍삳����@*/

     if(strcmp(line, "On")== 0){                 /* �T�[�{���[�^��On�ɂ���@*/
        std::cout << "SERVO ON" << std::endl;
        G_ROBO->setServo(1, 0);
     }else if(strcmp(line, "Off") == 0){         /* �T�[�{���[�^��Off�ɂ���@*/
        std::cout << "SERVO OFF" << std::endl;
        G_ROBO->setServo(0,0);
     }else if(strcmp(line, "Free") == 0){        /* ���ȏ�̕��ׂ�������΁A�T�[�{Off�ɂȂ郂�[�h�ֈڍs����@*/
        std::cout << "SERVO FREE" << std::endl;
        G_ROBO->setFreeMotion(1);
     }else if(strcmp(line, "Init") == 0){        /* �������p���i�����j�ɂȂ� */
        std::cout << "INIT" << std::endl;
        G_ROBO->initPosition();
     }else if(strcmp(line, "PrintMotion") == 0){ /* ���݂̓����\������@*/
        G_ROBO->record->printMotion();
     }else if(strcmp(line, "Start") == 0){       /* ���݂̓�������s�@*/
        std::cout << "START" << std::endl;
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "Reverse") == 0){     /* ���݂̓�����t���Ɏ��s�@*/
        std::cout << "REVERSE" << std::endl;
        G_ROBO->setMotionCount(-1);
     }else if(strcmp(line, "connect") == 0){     /* ���{�b�g�֍Đڑ�����@*/
        std::cout << "RECONNECT" << std::endl;
        G_ROBO->closePort();
        G_ROBO->connect();
     }else if(strcmp(line, "quit") == 0){        /* �R�}���h���̓��[�v�𔲂���(�I��) */
        loop = 0;
     }else{                                      /* ���͂��ꂽ������̓���p�^�[���t�@�C����T���āA���݂���Ύ��s���� */
        std::cout << "Motion = "<< line << std::endl;
        std::string file(line);

        if(G_ROBO->loadMotion((char *)file.c_str()) > 0){
           G_ROBO->setMotionCount(1);
        }
     }

     /* ���{�b�g�̎p���i���[�^�̊p�x�j�Ȃǂ�\������ */
     G_ROBO->printPosture();
   }

   /* �I�������@*/
   /** ���̃T�[�{���I�t�ɂ���i�X�C�b�`�������₷�����邽�߁j */
   G_ROBO->setServo(0, 1);

   /* GR001�̃N���X���폜���� */
   delete G_ROBO;

   std::cerr <<"Terminated." << std::endl;

   exit(0);
 }
