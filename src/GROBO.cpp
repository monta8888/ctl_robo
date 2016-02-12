/*
   Sample program to control G-001
  Copyright(C) 2011, Isao HARA, AIST,Japan

*/

#include "stdafx.h"
#include "GROBO.h"

static  unsigned char GR001_MoveJointPacket[] = {
      0x53, 0x0c, 0xfa, 0xaf, 0x03,
      0x00, 0x1e, 0x04, 0x01, 0xf6,
      0xff, 0x00, 0x00, 0x11
};

static unsigned char GR001_PosturePacket[] = {
       0x53 ,0x6c ,0xfa,0xaf ,0x00 ,0x00 ,0x1e ,0x05
      ,0x14
      ,0x01 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x02 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x03 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x04 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x05 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x06 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x07 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x08 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x09 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0a ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0b ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0c ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0d ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0e ,0x00 ,0x00 ,0x64 ,0x00
      ,0x0f ,0x00 ,0x00 ,0x64 ,0x00
      ,0x10 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x11 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x12 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x13 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x14 ,0x00 ,0x00 ,0x64 ,0x00
      ,0x1b
};


static unsigned char GR001_MotorInfoPacket[] = { 
    0x54, 0x09, 0xfa, 0xaf, 0x01,
    0x0f, 0x24, 0x12, 0x00, 0x26, 0x1a
};


static unsigned char GR001_ServoPacket[]={
   0x53, 0x09, 0xfa, 0xaf,
   0x01, 0x00,0x24, 0x01,
   0x01,0x01,0x26
};

static int init_pos[] ={
 0, 0, 
 200, 0, -200, 
 -200,0, 200, 
  0, 200, 0,-400, -200, 0,
  0, -200, 0, 400, 200, 0
};

static int limit_currents[] ={
 150, 50, 
 50, 50, 50, 
 50, 50, 50, 
  100, 180, 170, 200, 150, 160,
  100, 180, 170, 200, 150, 160
};
  /******* YAML  *****/

static const char jointIdToMotorIdMap[] = {
    9 /* R_HIP_Y */, 11 /* R_HIP_R */,  10 /* R_HIP_P */, 12 /* R_KNEE_P */, 13 /* R_ANKLE_P */, 14 /* R_ANKLE_R */,
    15 /* L_HIP_Y */, 17 /* L_HIP_R */, 16 /* L_HIP_P */, 18 /* L_KNEE_P */, 19 /* L_ANKLE_P */, 20 /* L_ANKLE_R */,
    1 /* CHEST_P */,  2 /* NECK_Y */,
    3 /* R_SHOULDER_P */, 4 /* R_SHOULDER_R */, 5 /* R_ELBOW_P */,
    6 /* R_SHOULDER_P */, 7 /* R_SHOULDER_R */, 8 /* R_ELBOW_P */ };

/**** Method for  GR001 ***/
GR001::GR001(char *devname): SerialRobot(devname, N_JOINTS)
{
  Currents= new unsigned short[N_JOINTS];
  Volts= new unsigned short[N_JOINTS];
  Speeds= new unsigned short[N_JOINTS];

  MoveJointPacket = new unsigned char[ sizeof(GR001_MoveJointPacket)];
  memcpy(MoveJointPacket, GR001_MoveJointPacket, sizeof(GR001_MoveJointPacket));

  PosturePacket   = new unsigned char [ sizeof(GR001_PosturePacket) ];
  memcpy(PosturePacket, GR001_PosturePacket, sizeof(GR001_PosturePacket));

  MotorInfoPacket = new unsigned char [ sizeof(GR001_MotorInfoPacket) ];
  memcpy(MotorInfoPacket, GR001_MotorInfoPacket, sizeof(GR001_MotorInfoPacket));

  ServoPacket     = new unsigned char [ sizeof(GR001_ServoPacket)];
  memcpy(ServoPacket, GR001_ServoPacket, sizeof(GR001_ServoPacket));

  record = new RobotMotion();
  record->setJoints(N_JOINTS);

  initPosture->setDegrees(init_pos);
}

GR001::~GR001()
{
  stopThread();


  delete Currents;
  delete Volts;
  delete Speeds;

  delete MoveJointPacket;
  delete PosturePacket;
  delete ServoPacket;
  delete MotorInfoPacket;

  delete record;

}


/*
  Set torque of a motor
*/
int 
GR001::setTorque( unsigned char id, char val)
{
  ServoPacket[4] = id;
  ServoPacket[9] = val;
  ServoPacket[10] = calcSum((char *)(&ServoPacket[4]), 6);

  if(sendCommand((char *)ServoPacket, 11) < 0){
#ifdef DEBUG_PRINT
    std::cerr<< " Fail to setTorque: id=" << std::dec << (int)id <<std::endl;
#endif
    return -1;
  }
  return 0;
}

/*

*/
unsigned char *
GR001::getMotorInfo(unsigned char id, unsigned char start_pos, unsigned char l)
{
  int c;
  unsigned char len;
  unsigned char *buf;

  len = l + 8;

  buf = new unsigned char[len];

  if(MotorInfoPacket == NULL){ return NULL; }
  MotorInfoPacket[4] = id;
  MotorInfoPacket[6] = start_pos;
  MotorInfoPacket[7] = l;

  MotorInfoPacket[9] = calcSum((char *)(&MotorInfoPacket[4]), 5);
  MotorInfoPacket[10] = len;

  if(sendCommand((char *)MotorInfoPacket, 11) < 0){
    std::cerr<< " :getMotorInfo" <<std::endl;
	delete buf;
	return NULL;
  }

  if((c = recieveData((char *)buf, (int)len)) < 0){
    delete buf;
    return NULL;
  }

  return buf;
}


/*
  Ask motor infomation, and set current position and voltages. 
*/
short
GR001::getAngle(unsigned char id)
{
  unsigned char *buf;
  buf = getMotorInfo(id, 0x24, 18);

  if(buf != NULL){
    short deg =  convC2S(&buf[13]);

    if(id == 12){ deg += 20; }
    if(id == 18){ deg -= 20; }

    currentPosture->setDegree(id, deg);
    setServoState(id-1, buf[7]);

    Currents[id-1] = convC2S(&buf[19]);
    Volts[id-1] = convC2S(&buf[23]);
	Speeds[id-1] = convC2S(&buf[17]);

    delete buf;
  }else{
#ifdef DEBUG_PRINT
	std::cerr<< "ERROR :getAngle:" << id <<std::endl;
#endif
	return -10000;
  }
  return currentPosture->getDegree(id);
 
}

/*
   getCurrent(int id)
*/
short
GR001::getCurrent(unsigned char id)
{
  unsigned char *buf;
  buf = getMotorInfo(id, 0x30, 2);
  if(buf != NULL){
    Currents[id-1] = convC2S(&buf[7]);
fprintf(stdout, "CUR[%02x %02x %02x %02x]\n", buf[0], buf[1], buf[2], buf[3]); // @@@
	delete buf;
  }else{
#ifdef DEBUG_PRINT
	std::cerr<< "ERROR :getCurrent" <<std::endl;
#endif
    return -10000;
  }

  return  Currents[id-1];
}



/*
   Turn on/off  all servos.
*/
void 
GR001::setServo(char on, int jointId)
{
  int i;
  int len=20;
  char ALL[20]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
  char body[]={HEAD, BODY};
  char r_arm[]={R_ARM0, R_ARM1, R_ARM2};
  char l_arm[]={L_ARM0, L_ARM1, L_ARM2};
  char arms[]={L_ARM0, L_ARM1, L_ARM2, R_ARM0, R_ARM1, R_ARM2};
  char r_leg[]={R_LEG0, R_LEG1, R_LEG2, R_KNEE, R_FOOT0, R_FOOT1};
  char l_leg[]={L_LEG0, L_LEG1, L_LEG2, L_KNEE, L_FOOT0, L_FOOT1};
  char legs[]={R_LEG0, R_LEG1, R_LEG2, R_KNEE, R_FOOT0, R_FOOT1, L_LEG0, L_LEG1, L_LEG2, L_KNEE, L_FOOT0, L_FOOT1};

  char *Target;

  Target = ALL;

  switch(jointId){
    case 1:
      Target = body;
      len = 2;
      break;
    case 2:
      Target = r_arm;
      len = 3;
      break;
    case 3:
      Target = l_arm;
      len = 3;
      break;
    case 4:
      Target = arms;
      len = 6;
      break;
    case 5:
      Target = r_leg;
      len = 6;
      break;
    case 6:
      Target = l_leg;
      len = 6;
      break;
    case 7:
      Target = legs;
      len = 12;
      break;
    default:
      break;
  }
  for(i=0; i < len; i++){
    setTorque( Target[i]+1, on);
	setServoState(Target[i], on);
  }
}

/*
  Move a joint
*/
int 
GR001::moveJoint(unsigned char id, short deg, unsigned short cs)
{

  MoveJointPacket[4] = id;
  setDegMs((char *)(&MoveJointPacket[9]), deg, cs);
  MoveJointPacket[13] = calcSum((char *)(&MoveJointPacket[4]), 9);

  if(sendCommand((char *)MoveJointPacket, 14) < 0){
    std::cerr << " :moveJoint" << std::endl;
  }
  return 0;
}

/****
  Set joint angles.
 ***/

int 
GR001::setJoints( short *deg, int cs)
{

  if(cs > 0){
    targetPosture->setMotionTime((double)cs * 10);
  }else{
    targetPosture->setMotionTime(0);
  }
  for(int i=0; i < numJoints() ; i++){
    targetPosture->setDegree(i+1, (int)deg[i]);
  }

  return 0;
}

int 
GR001::setJointsFromString(char *str)
{
  char *tp;
  short val;
  int i;

  tp = str;

  targetPosture->setMotionTime(500);

  getNumber(tp, &val);
  if(val > 0 && val < 1000){
    targetPosture->setMotionTime((double)val * 10);
  }

  for(i=0;i<joints;i++){
    tp = getNumber(tp, &val);
    if(tp == NULL) break;

     targetPosture->setDegree(i+1, (int)val);
  }
  return 0;
}


/*******/
int 
GR001::activateMotionSlot(unsigned char id)
{
  char cmd[]={0x50, 0x4d, 0x00};
  cmd[2] = id;

  if(sendCommand(cmd, 3) < 0){
  std::cerr << "Error in activateMotionSlot" <<std::endl;
    return -1;
  }
    
  return 1;
}

int 
GR001::activateSenario(unsigned char id)
{
  char cmd[]={0x50, 0x53, 0x00};
  cmd[2] = id;

  if(sendCommand(cmd, 3) < 0){
  std::cerr << "Error in activateSenario" << std::endl;
    return -1;
  }
    
  return 1;
}

/************************/


/*
   set target angles and movement time.
*/
int 
GR001::setDegMs(char *data, short deg, unsigned short cs)
{
  *data = deg & 0xff;
  *(data+1) = (deg >> 8) & 0xff;
  *(data+2) = cs & 0xff;
  *(data+3) = (cs >> 8) & 0xff;

  return 1;
}

/*
  print joint angles on a console.
*/
void 
GR001::printPosture()
{
#ifdef DEBUG_PRINT
  fprintf(stderr,"HEAD:              %5d\n", currentPosture->getDegree(2));

  fprintf(stderr," ARM: %5d %5d %5d -+",
    currentPosture->getDegree(8), 
    currentPosture->getDegree(7), 
    currentPosture->getDegree(6));
  fprintf(stderr,"- %5d %5d %5d \n", 
    currentPosture->getDegree(3),
    currentPosture->getDegree(4),
    currentPosture->getDegree(5));

  fprintf(stderr,"BODY:              %5d \n", currentPosture->getDegree(1));
  fprintf(stderr," LEG:         %5d     %5d  \n", 
    currentPosture->getDegree(15), currentPosture->getDegree(9));
  fprintf(stderr," LEG: %5d %5d %5d -+",
    currentPosture->getDegree(18), 
    currentPosture->getDegree(16),
    currentPosture->getDegree(17));
  fprintf(stderr,"- %5d %5d %5d \n",
    currentPosture->getDegree(11),
    currentPosture->getDegree(10), 
    currentPosture->getDegree(12));
  fprintf(stderr,"FOOT:    %5d %5d   -+",
    currentPosture->getDegree(20), currentPosture->getDegree(19));
  fprintf(stderr,"-  %d %5d \n\n", 
    currentPosture->getDegree(13), currentPosture->getDegree(14));

  printCurrents();
#endif
}

void 
GR001::printCurrents()
{
#if 0
  char body[]={HEAD, BODY};
  char r_arm[]={R_ARM0, R_ARM1, R_ARM2};
  char l_arm[]={L_ARM0, L_ARM1, L_ARM2};
  char arms[]={L_ARM0, L_ARM1, L_ARM2, R_ARM0, R_ARM1, R_ARM2};
  char r_leg[]={R_LEG0, R_LEG1, R_LEG2, R_KNEE, R_FOOT0, R_FOOT1};
  char l_leg[]={L_LEG0, L_LEG1, L_LEG2, L_KNEE, L_FOOT0, L_FOOT1};
  char legs[]={R_LEG0, R_LEG1, R_LEG2, R_KNEE, R_FOOT0, R_FOOT1, L_LEG0, L_LEG1, L_LEG2, L_KNEE, L_FOOT0, L_FOOT1};
#endif

  fprintf(stderr,"HEAD:%5d BODY: %5d\n", Currents[HEAD],Currents[BODY]);
  fprintf(stderr,"R_ARM:%5d %5d %5d\n", Currents[R_ARM0],Currents[R_ARM1],Currents[R_ARM2]);
  fprintf(stderr,"L_ARM:%5d %5d %5d\n", Currents[L_ARM0],Currents[L_ARM1],Currents[L_ARM2]);
  fprintf(stderr,"R_LEG:%5d %5d %5d %5d %5d %5d\n",
	  Currents[R_LEG0],Currents[R_LEG1],Currents[R_LEG2],Currents[R_KNEE],Currents[R_FOOT0],Currents[R_FOOT1]);
  fprintf(stderr,"L_LEG:%5d %5d %5d %5d %5d %5d\n",
	  Currents[L_LEG0],Currents[L_LEG1],Currents[L_LEG2],Currents[L_KNEE],Currents[L_FOOT0],Currents[L_FOOT1]);
  fprintf(stderr,"\n");
}

/*
    print Voltages on the motors.
*/
int 
GR001::printVolts(int flag)
{
  int i;
  int V=0;

  if(flag){
  std::cerr << "Volts: " ;
  }

  for(i=0; i<joints; i++){
  if(flag){
      std::cerr << Volts[i];
  }
    V += Volts[i];
  }
  if(flag){
   std::cerr << std::endl;
  }
  std::cerr << "Volts = " << (float)(V/200.0) << std::endl;

  return V;
}


int 
GR001::checkConnection()
{
#if 0 // @@@
  char buf;
  char msg1[] = {0x41, 0x00};
  if(sendCommand(msg1, 2) < 0){
    std::cerr << "Error in checkConnection" << std::endl;
    return -1;
  }

  Sleep(10);
  recieveData( &buf, 1);

  if(buf != 0x07){
    std::cerr << "Error in checkConnection : " << std::hex << std::showbase << buf << std::endl;
    return -1;
  }
#else
  char i;
  unsigned char msg1[] = {0x53, 0x09, 0xfa, 0xaf, 0x01, 0x00, 0x24, 0x01, 0x01, 0x01, 0x24};

  for (i=1; i<=20; i++) {
    msg1[4]  = i;
    msg1[10] = calcSum((char *)(&msg1[4]), 6);
#if 1
    if(sendCommand((char *)msg1, 11) < 0){
      std::cerr << "Error in checkServo" << std::endl;
      return -1;
    }
#else
    while( (sendCommand((char *)msg1, 11) < 0) || (getAngle(i)) == -10000) ){
      ;
    }
#endif
  }
#endif // @@@
  return 0;
}

/*

*/
int
GR001::connect(int n)
{
  int i;
  int c; 
  char cdev[32];

  if(n == 0){
    for(i=0;i<20;i++){

      SPRINTF(cdev, DEV_STR, i);
    setDevice(cdev);
      if((c=openPort()) < 0){
        continue; 
      }

      if(checkConnection() < 0){
        closePort();
        continue;
      }
    std::cerr << "Open Comm : " << cdev << std::endl;
      return 0;
    }
  }else{

    SPRINTF(cdev, DEV_STR, n); 
  setDevice(cdev);
    if(openPort()  < 0){
      std::cerr << "Fain to open " << cdev <<std::endl;
      return -1;
    }

    if(checkConnection() < 0){
      closePort();
      return -1;
    }

    return 0;
  }
  return -1;
}

/*************************/


bool 
GR001::servo(int jid, bool turnon)
{
  if(turnon){
    setServo(1, jid);

  }else{
    setServo(0, jid);
  }
  return true;
}

bool 
GR001::servo(const char *jname, bool turnon)
{
    std::map<std::string, int>::iterator l;
  if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        bool ret = true;
        for (int i=0; i<numJoints(); i++){
            ret = ret && servo(i, turnon);
        }
        return ret;
    }else if ((l = Link.find(jname)) != Link.end()) {
        return servo(l->second, turnon);
    }else{
        const std::vector<int> jgroup = m_jointGroups[jname];
        if (jgroup.size() == 0) return false;
        bool ret = true;
        for (unsigned int i=0; i<jgroup.size(); i++){
            ret = ret && servo(jgroup[i], turnon);
            return ret;
        }
    }
    return false;
}

void 
GR001::readJointAngles(double *o_angles){
  int i;
     for(i=0;i < numJoints(); i++){ 
    o_angles[i] = deg2rad(currentPosture->getDegree(i+1));;
  }
}

void 
GR001::writeJointCommands(const double *i_commands){
  int i;
  for(i=0;i < numJoints(); i++){
    short deg = rad2deg(i_commands[i+1]);
    setJoint((unsigned int)i, deg);
  }

}

bool 
GR001::checkEmergency(std::string &o_reason, int &o_id){
  return false;
}

void 
GR001::oneStep(){
  startMotion();
}

bool 
GR001::names2ids(const std::vector<std::string> &i_names, 
                            std::vector<int> &o_ids)
{ 
    bool ret = true;
    for (unsigned int i=0; i<i_names.size(); i++){
    std::map<std::string, int>::iterator l = Link.find(i_names[i].c_str());
        if (l == Link.end()){
            std::cout << "joint named [" << i_names[i] << "] not found" 
                      << std::endl;
            ret = false;
        }else{
            o_ids.push_back(l->second);
        }
    }
    return ret;
}

bool 
GR001::addJointGroup(const char *gname, const std::vector<std::string>& jnames)
{
    std::vector<int> jids;
    bool ret = names2ids(jnames, jids);
    m_jointGroups[gname] = jids;
    return ret;
}

void 
GR001::postureToCommand(RobotPosture *pos){

  int mt = targetPosture->motionTime;

  if(mt > 0 && mt < 10000){
	setTimeout( (int)targetPosture->motionTime );
  }else{
    setTimeout(getDefaultMotionTime());
  }

  for(int i=0; i<joints; i++){
    short posture = (short)pos->getDegree(i+1);
	short mtm = (short)getTimeout()/10;
//fprintf(stdout, "POS=%d TIM=%d ", posture, mtm);
    setDegMs((char *)(PosturePacket+i*5+10) , posture, mtm);
  }
//fprintf(stdout, "\n");

  PosturePacket[109] = calcSum((char *)(PosturePacket+4), 105);
  setCommand((char *)PosturePacket, 110);

  return;
}

int
GR001::connect(){
 return SerialRobot::connect();
}



int 
GR001::jointIdToMotorId(int jid)
{
	return jointIdToMotorIdMap[jid];
}

int 
GR001::motorIdToJointId(int mid)
{
	for(int i(0); i< N_JOINTS; i++){
		if (jointIdToMotorIdMap[i] == mid){
			return i;
		}
	}
	return 0;
}


double *
GR001::getCurrentJointAngles()
{
	RobotPosture *pos = getCurrentPosture();
	return pos->getJointAnglesRad();
}

int 
GR001::setTargetJointAngles(double *rad)
{
	int i;
	for(i=0;i<joints;i++){
		targetPosture->setDegree(i, rad2deg(rad[i]));
	}
	startMotion();
	return 0;
}

int 
GR001::stabilizer()
{
	return 0;
}


int 
GR001::clearRecord()
{
    record->clear();
	return 0;
}

int 
GR001::recordCurrentPosture()
{
	record->appendPosture(currentPosture->dupPosture());
	return 0;
}

int 
GR001::doRecord()
{
	clearMotion();
	motion->appendMotion(record);
	setMotionCount(1);
	return 0;
}

/*

test...

*/

int 
GR001::setFreeMotion(int x)
{
	freeMotion=x;
	return freeMotion;
}

#if 0 // @@@
// @@@
#include <sys/time.h>
#include <time.h>
static int xx=0;
static int yy=0;
#endif

void 
GR001::getPosture()
{
  int i;
  short val;
  //unsigned int IDS[] = {13,14,19,20};
  unsigned char IDS[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
  int max_ids = 20;
#if 0 // @@@
{
  struct timeval tv1;
  struct timeval tv2;
  struct timeval tv3;
  struct timeval tv4;
  struct timespec req;
  struct timespec rem;

  // timeval‚ð‰Šú‰»
  timerclear(&tv1);
  timerclear(&tv2);

#if 0
  // timeval‚ªÝ’è‚³‚ê‚Ä‚¢‚é‚©‚Ç‚¤‚©‚ð”»’è,•\Ž¦
  fprintf(stdout, "timerisset tv1 = %d / tv2 = %d\n",
          (timerisset(&tv1)),
          (timerisset(&tv2)));
#endif
xx++;
if (!(xx % 10)) { fprintf(stdout, "!!%d!!\n", yy); setTorque(1, (yy++ & 0x1)); }

  // timeval‚Å‚ ‚étv1‚ðÝ’è
  if (gettimeofday(&tv1, NULL)) { perror("failed to gettimeofday"); exit(1); }

  getCurrent(1);

  // timeval‚Å‚ ‚étv2‚ðÝ’è
  if (gettimeofday(&tv2, NULL)) { perror("failed to gettimeofday"); exit(1); }
#if 0
  // timerval‚Ì“à—e‚ð•\Ž¦
  fprintf(stdout, "tv1 sec = %ld / usec = %ld => %ld.%06ld\n",
          tv1.tv_sec,
          tv1.tv_usec,
          tv1.tv_sec,
          tv1.tv_usec
          );
  fprintf(stdout, "tv2 sec = %ld / usec = %ld => %ld.%06ld\n",
          tv2.tv_sec,
          tv2.tv_usec,
          tv2.tv_sec,
          tv2.tv_usec);
  // timercmp‚Åtimeval‚ð”äŠr
  char c = '=' + (timercmp(&tv1, &tv2, >)) - (timercmp(&tv1, &tv2, <));
  fprintf(stdout, "tv1 %c tv2\n", c);
#endif

  // timeval‚Ì·•ªŽZo
  timerclear(&tv3);
  timersub(&tv2, &tv1, &tv3);
//  fprintf(stdout, "tv2 - tv1 = %ld.%06ld\n",
  fprintf(stdout, "%ld.%06ld\n",
          tv3.tv_sec,
          tv3.tv_usec);
}
#else // @@@
  for(i = 0 ; i < max_ids ; i++){
	unsigned char id = IDS[i];
    if((val = getAngle(id)) == -10000){
       LOCK_COM
	   if(com->clearBuffer() < 0){
		   com->closePort();
		   UNLOCK_COM
		   return;
	   }
       UNLOCK_COM
	}else{
	  if (freeMotion == 1){
	    if(Currents[id-1] > limit_currents[id-1]){
		  setTorque(id, 2);
		  setServoState(id-1, 2);
	    }
	    if(Speeds[id-1] < 8){
		  setTorque(id, 1);
		  setServoState(id-1, 1);
	    }
	  }
	}

  }
#endif
  return; 
}
