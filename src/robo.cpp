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
   /*   変数の定義　*/
   char line[1024];

 #ifdef WIN32
    char *devname = "COM1";
 #else
    char *devname = "/dev/ttyMFD1";
// @@@    char *devname = "/dev/ttyUSB0";
 #endif

   /* Ctrl+C を押したときの処理を登録 */
   signal(SIGINT, sighandler);

   /* コマンドの引数でシリアルポートを指定する */
   if(argc > 1){
//     devname  = (char *)argv[1];
   }

   /* GR001のクラスの初期化とロボットへの接続　*/
   G_ROBO = new GR001((char *)devname);
   if( G_ROBO->connect() < 0 ){
     std::cerr << "Error: can't find G-Robot: " <<  devname << std::endl;
   }
   if(G_ROBO->startThread() == 0){
     std::cerr << "Error: fail to create thread" << std::endl;
     delete G_ROBO;
     exit(1);
   }
   if(G_ROBO->startThread2() == 0){ // @@@
     std::cerr << "Error: fail to create thread2" << std::endl;
   }

   /* 動作ファイルのディレクトリを指定する　*/
   G_ROBO->setMotionDir("motion");

   if(argc > 1){
     if      (strcmp(argv[1], "f") == 0) { /* Forward */
        std::cout << "FORWARD" << std::endl;
        G_ROBO->selectMove(1);
        G_ROBO->setMotionCount(1);
        while(G_ROBO->isMoving());
        loop = 0;
     }else if(strcmp(argv[1], "b") == 0) { /* Back */
        std::cout << "BACK" << std::endl;
        G_ROBO->selectMove(2);
        G_ROBO->setMotionCount(1);
        while(G_ROBO->isMoving());
        loop = 0;
     }else if(strcmp(argv[1], "a") == 0) { /* Left */
        std::cout << "LEFT" << std::endl;
        G_ROBO->selectMove(3);
        G_ROBO->setMotionCount(1);
        while(G_ROBO->isMoving());
        loop = 0;
     }else if(strcmp(argv[1], "j") == 0) { /* Right */
        std::cout << "RIGHT" << std::endl;
        G_ROBO->selectMove(4);
        G_ROBO->setMotionCount(1);
        while(G_ROBO->isMoving());
        loop = 0;
     }else if(strcmp(argv[1], "i") == 0){ /* 初期化姿勢（中腰）になる */
        std::cout << "INIT" << std::endl;
        G_ROBO->initPosition();
        while(G_ROBO->isMoving());
        loop = 0;
     }else if(strcmp(argv[1], "q") == 0){ /* コマンド入力ループを抜ける(終了) */
        loop = 0;
     }
     /* ロボットの姿勢（モータの角度）などを表示する */
     G_ROBO->printPosture();
   }
   /* コマンド入力ループ　*/
   while(loop){
     /* コマンド入力プロンプトを出して、一行の入力を待つ */
     std::cout << "G-001 >";
     scanf("%s",line);

    /* 入力されたコマンド文字列に応じて、ロボットを動作させる　*/

     if(strcmp(line, "On")== 0){                 /* サーボモータをOnにする　*/
        std::cout << "SERVO ON" << std::endl;
        G_ROBO->setServo(1, 0);
     }else if(strcmp(line, "Off") == 0){         /* サーボモータをOffにする　*/
        std::cout << "SERVO OFF" << std::endl;
        G_ROBO->setServo(0,0);
     }else if(strcmp(line, "Free") == 0){        /* 一定以上の負荷がかかれば、サーボOffになるモードへ移行する　*/
        std::cout << "SERVO FREE" << std::endl;
        G_ROBO->setFreeMotion(1);
     }else if(strcmp(line, "i") == 0){        /* 初期化姿勢（中腰）になる */
        std::cout << "INIT" << std::endl;
        G_ROBO->initPosition();
     }else if(strcmp(line, "PrintMotion") == 0){ /* 現在の動作を表示する　*/
        G_ROBO->record->printMotion();
     }else if(strcmp(line, "Start") == 0){       /* 現在の動作を実行　*/
        std::cout << "START" << std::endl;
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "Reverse") == 0){     /* 現在の動作を逆順に実行　*/
        std::cout << "REVERSE" << std::endl;
        G_ROBO->setMotionCount(-1);
     }else if(strcmp(line, "connect") == 0){     /* ロボットへ再接続する　*/
        std::cout << "RECONNECT" << std::endl;
        G_ROBO->closePort();
        G_ROBO->connect();
     }else if(strcmp(line, "f") == 0) { /* Forward */
        std::cout << "[[FORWARD]]" << std::endl;
        G_ROBO->selectMove(1);
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "b") == 0) { /* Back */
        std::cout << "[[BACK]]" << std::endl;
        G_ROBO->selectMove(2);
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "a") == 0) { /* Left */
        std::cout << "[[LEFT]]" << std::endl;
        G_ROBO->selectMove(3);
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "j") == 0) { /* Right */
        std::cout << "[[RIGHT]]" << std::endl;
        G_ROBO->selectMove(4);
        G_ROBO->setMotionCount(1);
     }else if(strcmp(line, "q") == 0){        /* コマンド入力ループを抜ける(終了) */
        loop = 0;
     }else{                                      /* 入力された文字列の動作パターンファイルを探して、存在すれば実行する */
        std::cout << "Motion = "<< line << std::endl;
        std::string file(line);

        if(G_ROBO->loadMotion((char *)file.c_str()) > 0){
           G_ROBO->setMotionCount(1);
        }
     }

     /* ロボットの姿勢（モータの角度）などを表示する */
     G_ROBO->printPosture();
   }

   /* 終了処理　*/
   /** 胸のサーボをオフにする（スイッチを押しやすくするため） */
   G_ROBO->setServo(0, 1);

   /* GR001のクラスを削除する */
   delete G_ROBO;

   std::cerr <<"Terminated." << std::endl;

   exit(0);
 }
