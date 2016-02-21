/*
  Sample program to control G-001
  Copyright(C) all right reserved 2011, Isao HARA, AIST,Japan

*/
#ifndef __G_ROBOT_H__
#define __G_ROBOT_H__


#include "SerialRobot.h"
#include <string>
#include <vector>
#include <map>

#define N_JOINTS  20
#define MAX_CMD_LEN  127

/** Moter IDs */
#define HEAD 1
#define BODY 0

#define R_ARM0 2
#define R_ARM1 3
#define R_ARM2 4

#define L_ARM0 5
#define L_ARM1 6
#define L_ARM2 7

#define R_LEG0 8
#define R_LEG1 10
#define R_LEG2 9

#define R_KNEE 11

#define R_FOOT0 12
#define R_FOOT1 13

#define L_LEG0 14
#define L_LEG1 16
#define L_LEG2 15
#define L_KNEE 17

#define L_FOOT0 18
#define L_FOOT1 19

/*
 *  Control class for GR001:G-ROBOT
 * 
 */
#ifdef WIN32
class __declspec(dllexport) GR001: public SerialRobot
#else
class GR001: public SerialRobot
#endif
{
public:
  /*
   *Constructor
   */
  GR001(char *devname, int brate);
  /*
   * Deconstructor
   */
  ~GR001();

  /*
   * connect to the robot
   */
  int connect(int n);
  int connect();
  /*
   *  check connection
   */
  int checkConnection();

  /*
   *  output the current posture.
   */
  void printPosture();
  void printCurrents();
  int printVolts(int flag);

  /*
   * set servo
   */
  int setTorque(unsigned char id, char val);
  void setServo(char on, int c);
  int changeBaudRate(int n);

  /*
   *  get the angle of the motor.
   */
  short getAngle(unsigned char id);

  short getCurrent(unsigned char id);

  unsigned char *getMotorInfo(unsigned char id, unsigned char start_pos, unsigned char l);

  /*
   * set target degrees and time to the command packet.
   */
  int setDegMs(char *data, short deg, unsigned short cs);
  /*
   * move a joint.
   */
  int moveJoint(unsigned char id, short deg, unsigned short cs);

  /*
   * set joint angles and motionTime to the 'targetPosture'.
   */
  int setJoints(short *deg, int cs);
  /*
   * set joint angles to the 'targetPosture' from a string.
   */
  int setJointsFromString(char *str);

  /*
   * activate motion slot
   */
  int activateMotionSlot(unsigned char id);

  /*
   * activate senario.
   */
  int activateSenario(unsigned char id);

  /*
   *  convert a RobotPosture to a command packet.  
   */
  void postureToCommand(RobotPosture *pos);

  double *getCurrentJointAngles();

  int setTargetJointAngles(double *rad);

  int clearRecord();

  int recordCurrentPosture();

  int doRecord();

  void getPosture();

       /**  for hrpsys **/
  bool servo(int jid, bool turnon);
  bool servo(const char *jname, bool turnon);
  void readJointAngles(double *o_angles);
  void writeJointCommands(const double *i_commands);
  bool checkEmergency(std::string &o_reason, int &o_id);
  void oneStep();
  bool addJointGroup(const char *gname, const std::vector<std::string>& jnames);


  int jointIdToMotorId(int jid);
  int motorIdToJointId(int mid);
  int stabilizer();

  int setFreeMotion(int x);

private:
  bool names2ids(const std::vector<std::string> &i_names, std::vector<int> &o_ids);

public:
  RobotMotion *record;

private:
  unsigned short *Currents;
  unsigned short *Volts;
  unsigned short *Speeds;

  unsigned char *MoveJointPacket;     // a command packet template to move a joint.
  unsigned char *PosturePacket;       // a command packet template to move joints.
  unsigned char *MotorInfoPacket;     // a command packet template to request joint angles 
  unsigned char *ServoPacket;         // a command packet template to set a servo state.
  std::map<std::string, std::vector<int> > m_jointGroups;  // table of joint group.
  std::map<std::string, int> Link;    // table of links.

  int freeMotion;



};

#endif

