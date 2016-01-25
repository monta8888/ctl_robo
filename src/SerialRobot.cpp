/*
  Robot control class with serial commnication.

  Copyright (C) all right reserved. 2011 Isao Hara, AIST, Japan.

*/

#include "stdafx.h"
#include "SerialRobot.h"
#include <fstream>
#include <sstream>

#include "YamlParser.h"


/*
 RobotPosture Class
*/

/*
  Constructor 
*/
RobotPosture::RobotPosture(int n)
{
  numJoints      = n;
  jointAngles    = new int[n];
  jointAnglesRad = new double[n];
  motionTime     = 250;

  for(int i=0;i<n;i++){
    jointAngles[i]=0;
    jointAnglesRad[i]=0.0;
  }
}

/*
   Destructor
*/
RobotPosture::~RobotPosture()
{
  delete jointAngles;
  delete jointAnglesRad;
}

/*
  Duplicate RobotPosture
*/

RobotPosture *
RobotPosture::dupPosture()
{
  RobotPosture *res = new RobotPosture(numJoints);
  res->motionTime = motionTime;
  for(int i=0;i<numJoints;i++){
    res->jointAngles[i] = jointAngles[i];
    res->jointAnglesRad[i] = jointAnglesRad[i];
  }
  return res;
}

/*
  Copy posture from 'p'
*/
void 
RobotPosture::copyPosture(RobotPosture *p)
{
  copyPosture(p,0);
}

void 
RobotPosture::copyPosture(RobotPosture *p, int offset)
{
  int i;

  for(i=0; i<numJoints-offset && p->numJoints;i++){
    jointAngles[i+offset] = p->jointAngles[i];
    jointAnglesRad[i+offset] = p->jointAnglesRad[i];
  }
  setMotionTime(p->motionTime);
  return;
}


/*
 set motionTime
*/
double 
RobotPosture::setMotionTime(double n)
{
  motionTime = n;
  return motionTime;
}
/*
   
*/
int 
RobotPosture::getDegree(int id)
{
  if(id > 0 && id <= numJoints){
    return jointAngles[id - 1];
  }
  return -10000;
}

double 
RobotPosture::getRad(int id)
{
  if(id > 0 && id <= numJoints){
    return jointAnglesRad[id - 1];
  }
  return -10000.0;
}

void 
RobotPosture::setDegree(int id, int deg){
  if(id > 0 && id <= numJoints){

    jointAngles[id - 1] = deg;
  }
  return;
}

void 
RobotPosture::setRad(int id, double rad){
  if(id > 0 && id <= numJoints){
    jointAnglesRad[id - 1] = rad;
  }
  return;
}


bool
RobotPosture::setDegrees(int *degs)
{
  for(int i=0;i<numJoints;i++){
	  jointAngles[i] = degs[i];
  }
  return true;
}

/*

*/
int *
RobotPosture::getJointAngles()
{
	return jointAngles;
}

/*
  convert jointAngles to radian expression and copy it to jointAnglesRad

*/
double *
RobotPosture::getJointAnglesRad()
{
  int i;

  for(i=0;i<numJoints;i++){
    jointAnglesRad[i] = deg2rad(jointAngles[i]);
  }
  return jointAnglesRad;
}



/*
  output Posture
*/
void
RobotPosture::printPosture()
{
  int i;
  for(i=0;i<numJoints;i++){
    std::cerr << std::dec << " " << jointAngles[i];
  }
  std::cerr << std::endl;
}

void
RobotPosture::printPosture(std::ofstream& ofs)
{
  int i;

  for(i=0;i<numJoints;i++){
    ofs << " " <<jointAngles[i];
  }
  ofs << std::endl;
}

/*
  compare RobotPosture
*/

bool
RobotPosture::equalTo(RobotPosture *pos)
{
  if(pos == NULL) { return false; }
  if (pos->numJoints != numJoints){ return false; }

  for(int i=0;i<numJoints;i++){
	  if(pos->jointAngles[i] != jointAngles[i]) { return false; }
  }
  return true;
}

bool 
RobotPosture::nearTo(RobotPosture *pos, int delta)
{
  if(pos == NULL) { return false; }
  if (pos->numJoints != numJoints){ return false; }

  for(int i=0;i<numJoints;i++){
	  if(abs(pos->jointAngles[i] - jointAngles[i]) > delta) { return false; }
  }
  return true;
}


/*
  RobotMotion

*/
/*
   Constructor
*/
RobotMotion::RobotMotion() : numJoints(0),current(0),reverse(false)
{

}

/*
   Destructor
*/
RobotMotion::~RobotMotion(){
  int i;
  int n = motion.size();
  for(i=0;i<n;i++){
    delete motion[i];
  }
}

/*
   set a number of joints.
*/
int 
RobotMotion::setJoints(int n)
{
  numJoints = n;
  return n;
}

/*
  get size of motion vector.
*/
int 
RobotMotion::getSize(){
  return motion.size();
}

/*
   set revers flag to 'flag'
*/
int 
RobotMotion::setReverse(bool flag){
  reverse = flag;
  return reset();
}

/*
    reset current state
*/
int 
RobotMotion::reset()
{
  if(reverse){
    current = motion.size() - 1;
  }else{
    current = 0;
  }
  return current;
}

/*
  return a next posture
*/
RobotPosture *
RobotMotion::next()
{
  RobotPosture *res = get(current);
  if(reverse){
    current = (current + motion.size() - 1) % motion.size();
  }else{
    current = (current + 1) % motion.size();
  }
  return res;
  
}

/*
   return an N-th posture
*/
RobotPosture *
RobotMotion::get(int nth){
  return motion.at(nth);
}

/*
  return an N-th posture from the last.
*/
RobotPosture *
RobotMotion::rget(int nth){
  int n = motion.size();
  return motion.at(n - nth);
}

/*
   append a posture on the last of a motion.
*/

bool 
RobotMotion::appendPosture(RobotPosture *js)
{
  motion.push_back(js);
  return true;
}

/*
   insert a posture into the N-th of motion vector.
*/
bool 
RobotMotion::insertPosture(RobotPosture *js, int nth){
  int i;
  std::vector<RobotPosture *>::iterator  it;

  for(i=0,it=motion.begin(); it != motion.end() ;i++){ 
    if(i >= nth ) break;
    it++;
  }
  
  motion.insert(it, js);
  return true;
}


bool 
RobotMotion::appendMotion(RobotMotion *rm)
{
	int len = rm->getSize();
	for(int i(0); i<len; i++){
		motion.push_back(rm->get(i));
	}
    return true;
}

RobotMotion * 
RobotMotion::dupMotion()
{
	RobotMotion *rm = new RobotMotion();
	rm->setJoints(numJoints);

	for(int i(0); i<getSize(); i++){
		rm->motion.push_back(get(i));
	}
    return rm;
}
/*
  delete an N-th posture from the motion vector.
*/
void 
RobotMotion::deletePosture(int nth){
  int i;
  std::vector<RobotPosture *>::iterator  it;

  for(i=0,it=motion.begin(); it != motion.end() ;i++){ 
    if(i >= nth ) break;
    it++;
  }
  
  delete it[0];
  motion.erase(it);

  return;
}

/*
  Clear the current motion vector
*/

void
RobotMotion::clear()
{
  int n = motion.size();
 
  for(int i=0; i<n; i++){
    deletePosture(0);
  }
  return;
}


/*
  Load/Save Robot Motion from file.

*/
/** Load **/
bool 
RobotMotion::loadMotionFromFile(const char *name)
{
  std::ifstream ifs;
  ifs.open(name);

  if(! ifs.fail()){
    clear();
    double st = 0;
    char line[1024];

    while( !ifs.eof() ){
      double mt;
      ifs.getline(line,1024);
      std::istringstream sfs(line);
      sfs >> mt;
      if(sfs.fail()) { break; }
      RobotPosture *rp = new RobotPosture(numJoints);
      rp->motionTime = mt - st;
      st = mt;
      for(int i=0;i < numJoints;i++){
        int deg;
        sfs >> deg;
        if(sfs.fail()) { deg = 0; }
        rp->setDegree(i+1, deg);
      }
      motion.push_back( rp );
    }
    ifs.close();
        return true;
  }else{
    std::cerr << "Faile to open " << name << std::endl;
  }

  return false;
}

bool 
RobotMotion::loadMotionFromMseqFile(const char *name, SerialRobot *r)
{

  YamlDocument doc;
  YamlMapping *topMap;
  YamlSequence *refs;

  if(!doc.load((char *)name)){
	  return false;
  }

  if(doc.empty() ){
    return false;
  }

  topMap = doc.root->childNodes[0]->toMapping();

  if( topMap == NULL  
	|| !topMap->equalTo("type", "MotionSeq")
	|| (refs = topMap->getSequence("refs")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }

  clear();

  int len = refs->size();
  double st = 0;
  double mt;
  short deg = 0;

  for(int i(0); i< len;i++){
    RobotPosture *rp;

	YamlMapping *ref_i = refs->getMappingAt(i);

	if(ref_i == NULL || ref_i->getScalar("time") == NULL)
	{
		continue;
	}
 
	mt = ref_i->getScalar("time")->toInteger();

	if(i==0){
      //  rp = new RobotPosture(numJoints);
		rp = r->currentPosture->dupPosture();
	}else{
		rp = motion[i-1]->dupPosture();
	}

    rp->motionTime = (int)((mt - st)*1000); /* sec -> msec */
    st = mt;

	YamlSequence *joint_seq, *q_seq;

	if( ref_i->getMapping("refer") == NULL || 
		(joint_seq = ref_i->getMapping("refer")->getSequence("joints")) == NULL ||
		(q_seq = ref_i->getMapping("refer")->getSequence("q")) == NULL )
	{
		continue;
	}

	int joints = joint_seq->size();

    for(int j=0;j < joints;j++){
		if(joint_seq->getScalarAt(j) == NULL ||
			q_seq->getScalarAt(j) == NULL)
		{
			continue;
		}
		int jid = joint_seq->getScalarAt(j)->toInteger();
		double rad = q_seq->getScalarAt(j)->toDouble();

		int idx = r->jointIdToMotorId(jid);
		short deg0 = rp->getDegree(idx);
		short deg = rad2deg(rad)*10;
        rp->setDegree(idx, deg0 + deg);
    }
    motion.push_back( rp );
  }
  return true;
}


bool 
RobotMotion::loadMotionFromPseqFile(const char *name, SerialRobot *r)
{

  YamlDocument doc;
  YamlMapping *topMap;
  YamlSequence *refs;

  if(!doc.load((char *)name)){
	  return false;
  }

  if(doc.empty() ){
    return false;
  }

  topMap = doc.root->childNodes[0]->toMapping();

  if( topMap == NULL  
	|| !topMap->equalTo("type", "PoseSeq")
	|| (refs = topMap->getSequence("refs")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }

  clear();

  int len = refs->size();
  double st = 0;
  double mt;
  short deg = 0;

  for(int i(0); i< len;i++){
    RobotPosture *rp;

	YamlMapping *ref_i = refs->getMappingAt(i);

	if(ref_i == NULL || ref_i->getScalar("time") == NULL)
	{
		continue;
	}
 
	mt = ref_i->getScalar("time")->toInteger();

	if(i==0){
      //  rp = new RobotPosture(numJoints);
		rp = r->currentPosture->dupPosture();
	}else{
		rp = motion[i-1]->dupPosture();
	}

    rp->motionTime = (int)((mt - st)*1000); /* sec -> msec */
//std::cerr << "mt=" << mt << "st = "<< st << std::endl; // @@@
    st = mt;

	YamlSequence *joint_seq, *q_seq;

	if( ref_i->getMapping("refer") == NULL || 
		(joint_seq = ref_i->getMapping("refer")->getSequence("joints")) == NULL ||
		(q_seq = ref_i->getMapping("refer")->getSequence("q")) == NULL )
	{
		continue;
	}

	int joints = joint_seq->size();

    for(int j=0;j < joints;j++){
		if(joint_seq->getScalarAt(j) == NULL ||
			q_seq->getScalarAt(j) == NULL)
		{
			continue;
		}
		int jid = joint_seq->getScalarAt(j)->toInteger();
		double rad = q_seq->getScalarAt(j)->toDouble();

		int idx = r->jointIdToMotorId(jid);
		short deg = rad2deg(rad)*10;
        rp->setDegree(idx, deg );
//std::cerr << "idx=" << idx << "deg = "<< deg << std::endl; // @@@
    }
    motion.push_back( rp );
  }
  return true;
}



bool 
RobotMotion::loadMotionFromYamlFile(const char *name, SerialRobot *r)
{
  YamlDocument doc;
  YamlMapping *topMap;
  YamlSequence *comps, *frames;

  if(!doc.load((char *)name)){
	  std::cerr << "Fail to loadMotionFromYamlFile" << std::endl;
	  return false;
  }

  if(doc.empty() ){
    return false;
  }

  topMap = doc.root->childNodes[0]->toMapping();

  if( topMap == NULL  
	|| !topMap->equalTo("type", "BodyMotion")
	|| (comps = topMap->getSequence("components")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }
 
  clear();

  for( int i(0); i<comps->size() ;i++){
	YamlMapping *m = comps->at(i)->toMapping();

	if(m == NULL) continue;
	if( m->equalTo("type", "MultiValueSeq") && 
			m->equalTo("purpose", "JointPosition") )
	{
      int frate, nFrames;

	  if(m->getScalar("frameRate") == NULL ){
		 frate = 10;
	  }else{
         frate = m->getScalar("frameRate")->toInteger();
	  }
	  if(m->getScalar("numFrames") == NULL){
        nFrames = 0;
	  }else{
		nFrames = m->getScalar("numFrames")->toInteger();
	  }

      double tm = 1000.0 / frate;
      short deg=0;

      frames = m->getSequence("frames");
	  if(frames == NULL){
		  break;
	  }

      for(int j(0); j < nFrames ;j++){
        RobotPosture *rp;
		int joints;
        rp = new RobotPosture(numJoints);

        rp->motionTime = (int)tm; /* sec -> msec */

		if(frames->getSequenceAt(j) == NULL){
		  joints = 0;
		}else{
		  joints = frames->getSequenceAt(j)->size();
		}

        for(int k(0) ; k < joints; k++){
		  double rad = 0;
		  short deg = 0;
          int idx = r->jointIdToMotorId(k);
		  if(frames->getSequenceAt(j)->getScalarAt(k) != NULL){
		    rad = frames->getSequenceAt(j)->getScalarAt(k)->toDouble();
			deg = rad2deg(rad)*10;
		  }
          rp->setDegree(idx, deg );
        }
        motion.push_back( rp );
      }
	  break;
	}
  }

  if( motion.size() > 0){
	return true;
  }else{
	  return false;
  }
}


/** Save **/
bool 
RobotMotion::saveMotionToFile(const char *name)
{
  std::ofstream ofs;

  ofs.open(name);

  if(!ofs.fail()){
    printMotion(ofs);
  }else{
    std::cerr << "Faile to open " << name << std::endl;
  }
  ofs.close();
  return true;
}

bool 
RobotMotion::saveMotionToFile(const char *fname, const char *dir)
{
	int i=0;
	char fileName[128];
	sprintf(fileName, "%s%d",fname, i);

	std::string filename;
	if(strlen(dir) > 0){
		filename = std::string(dir) + FileDelim + fileName;
	}else{
		filename =  fileName;
	}
	
	while(FileExists(filename.c_str(),"m")){
	  i++;
	  sprintf(fileName, "%s%d", fname, i);
	  filename = std::string(dir) + FileDelim + fileName;
	}
	sprintf(fileName, "%s%d.m",fname, i);
	filename = std::string(dir) + FileDelim + fileName;
	return saveMotionToFile(filename.c_str());
}

bool 
RobotMotion::saveMotionToPseqFile(const char *name, SerialRobot *r)
{
  std::cerr << "saveMotionToPseqFile isn't implemented." << std::endl;
  return false;
}

bool 
RobotMotion::saveMotionToYamlFile(const char *name, SerialRobot *r)
{
  std::cerr << "saveMotionToYamlFile isn't implemented." << std::endl;
  return false;
}

/*
  Output motion vector (for DEBUG)
*/

void
RobotMotion::printMotion()
{
  int i, n;
  n = motion.size();

  for( i=0; i<n ; i++){
   std::cerr << "Posture[ " << i << " ] = " << motion.at(i)->motionTime << ":";
   motion.at(i)->printPosture();
  }

  return;
}

void
RobotMotion::printMotion(std::ofstream& ofs)
{
  int i, n;
  double tm;
  n = motion.size();


  for( i=0,tm=0; i<n ; i++){
   tm += motion.at(i)->motionTime;
     ofs << tm << " " ;
   motion.at(i)->printPosture(ofs);

  }

  return;
}

/*
  S01腰
  S02頭
  S03右肩
  S04右二の腕
  S05右腕
  S06左肩
  S07左二の腕
  S08左腕
  S09右付け根
  S10右太もも前
  S11右太もも後
  S12右膝
  S13右足首前
  S14右足首後
  S15左付け根
  S16左太もも前
  S17左太もも後
  S18左膝
  S19左足首前
  S20左足首後
 */
// @@@
void
RobotMotion::setForward( SerialRobot *r )
{
  int pos_forward[20][20] ={ // 前進.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0,-250,  50,  500,-250,  50,  500,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200,-100}, // Soc
    {      0,   0,-250,   0, 1200,-250,   0, 1200,   0,   0, 350,-450, 150,   0,   0,   0, 350,-450, 150,   0}, // 00
    {      0,   0,-400,   0, 1200, -50,   0, 1200,   0,  50, 450,-650, 280,  20,   0,   0, 200,-350, 200,  20}, // 01
    {      0,   0,-250,   0, 1200,-250,   0, 1300,   0,   0, 400,-350,   0,   0,   0,   0, 200,-350, 200,   0}, // 03
    {      0,   0, -50,   0, 1200, -50,   0, 1300,   0,   0, 200,-350, 200,  20,   0,  50, 450,-650, 220,  40}, // 02
    {      0,   0,-250,   0, 1200,-250,   0, 1300,   0,   0, 200,-350, 200,   0,   0,   0, 400,-350,   0,   0}, // 04
    {      0,   0,-250,   0, 1200,-250,   0, 1200,   0,   0, 350,-450, 150,   0,   0,   0, 350,-450, 150,   0}, // 00
#else /*          ±逆       ±逆                        ←逆→                       ←逆→ */
    {      0,   0, 250,  50, -500,-250,  50,  500,   0, 400, 100,-550,-200,-100,   0,-400,-100, 550, 200,-100}, // Soc
    {      0,   0, 250,   0,-1200,-250,   0, 1200,   0, 350,   0,-450,-150,   0,   0,-350,   0, 450, 150,   0}, // 00
    {      0,   0, 400,   0,-1200, -50,   0, 1200,   0, 450,  50,-650,-280, -20,   0,-200,   0, 350, 200,  20}, // 01
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 400,   0,-350,   0,   0,   0,-200,   0, 350, 200,   0}, // 03
    {      0,   0,  50,   0,-1200, -50,   0, 1300,   0, 200,   0,-350,-200, -20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 200,   0,-350,-200,   0,   0,-400,   0, 350,   0,   0}, // 04
    {      0,   0, 400,   0,-1200, -50,   0, 1200,   0, 450,  50,-650,-280, -20,   0,-200,   0, 350, 200,  20}, // 01
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 400,   0,-350,   0,   0,   0,-200,   0, 350, 200,   0}, // 03
    {      0,   0,  50,   0,-1200, -50,   0, 1300,   0, 200,   0,-350,-200, -20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 200,   0,-350,-200,   0,   0,-400,   0, 350,   0,   0}, // 04
    {      0,   0, 400,   0,-1200, -50,   0, 1200,   0, 450,  50,-650,-280, -20,   0,-200,   0, 350, 200,  20}, // 01
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 400,   0,-350,   0,   0,   0,-200,   0, 350, 200,   0}, // 03
    {      0,   0,  50,   0,-1200, -50,   0, 1300,   0, 200,   0,-350,-200, -20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 200,   0,-350,-200,   0,   0,-400,   0, 350,   0,   0}, // 04
    {      0,   0, 400,   0,-1200, -50,   0, 1200,   0, 450,  50,-650,-280, -20,   0,-200,   0, 350, 200,  20}, // 01
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 400,   0,-350,   0,   0,   0,-200,   0, 350, 200,   0}, // 03
    {      0,   0,  50,   0,-1200, -50,   0, 1300,   0, 200,   0,-350,-200, -20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0, 250,   0,-1200,-250,   0, 1300,   0, 200,   0,-350,-200,   0,   0,-400,   0, 350,   0,   0}, // 04
    {      0,   0, 400,   0,-1200, -50,   0, 1200,   0, 450,  50,-650,-280, -20,   0,-200,   0, 350, 200,  20}, // 01
    {      0,   0, 250,   0,-1200,-250,   0, 1200,   0, 350,   0,-450,-150,   0,   0,-350,   0, 450, 150,   0}, // 00
#endif                                                                 ↑±逆↑       ↑±逆↑  ↑
  };
  int len = 20;
  double mt = 1;

  clear();

  for(int i(0); i< len;i++){
    RobotPosture *rp;
    if(i==0){
      rp = r->currentPosture->dupPosture();
    }else{
      rp = motion[i-1]->dupPosture();
    }
    rp->motionTime = (int)mt*100; /* sec -> msec */
    rp->setDegrees(pos_forward[i]);
//rp->printPosture();
    motion.push_back( rp );
  }
  return;
}

void
RobotMotion::setBack( SerialRobot *r )
{
  int pos_back[20][20] ={ // 後退.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0,-250,  50,  500,-250,  50,  500,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200,-100}, // Soc
    {      0,   0, 100,   0, 1200, 100,   0, 1200,   0,   0, 300,-350, 100,   0,   0,   0, 300,-350, 100,   0}, // 00
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,  50, 400,-650, 280, -20,   0,   0, 350,-350,  50,  20}, // 01
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 250,-350, 150,   0,   0,   0, 350,-350,  50,   0}, // 03
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 350,-350,  50,  20,   0,  50, 450,-650, 220,  40}, // 02
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 350,-350,  50,   0,   0,   0, 250,-350, 150,   0}, // 04
    {      0,   0, 100,   0, 1200, 100,   0, 1200,   0,   0, 300,-350, 100,   0,   0,   0, 300,-350, 100,   0}, // 00
#else /*          ±逆       ±逆                        ←逆→                       ←逆→ */
    {      0,   0, 250,  50, -500,-250,  50,  500,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200,-100}, // Soc
    {      0,   0,-100,   0,-1200, 100,   0, 1200,   0, 300,   0,-350,-100,   0,   0,-300,   0, 350, 100,   0}, // 00
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 400,  50,-650,-280,  20,   0,-350,   0, 350,  50,  20}, // 01
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 250,   0,-350,-150,   0,   0,-350,   0, 350,  50,   0}, // 03
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,  20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,   0,   0,-250,   0, 350, 150,   0}, // 04
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 400,  50,-650,-280,  20,   0,-350,   0, 350,  50,  20}, // 01
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 250,   0,-350,-150,   0,   0,-350,   0, 350,  50,   0}, // 03
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,  20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,   0,   0,-250,   0, 350, 150,   0}, // 04
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 400,  50,-650,-280,  20,   0,-350,   0, 350,  50,  20}, // 01
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 250,   0,-350,-150,   0,   0,-350,   0, 350,  50,   0}, // 03
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,  20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,   0,   0,-250,   0, 350, 150,   0}, // 04
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 400,  50,-650,-280,  20,   0,-350,   0, 350,  50,  20}, // 01
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 250,   0,-350,-150,   0,   0,-350,   0, 350,  50,   0}, // 03
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,  20,   0,-450, -50, 650, 220,  40}, // 02
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 350,   0,-350, -50,   0,   0,-250,   0, 350, 150,   0}, // 04
    {      0,   0,   0,   0,-1200,   0,   0, 1200,   0, 400,  50,-650,-280,  20,   0,-350,   0, 350,  50,  20}, // 01
    {      0,   0,-100,   0,-1200, 100,   0, 1200,   0, 300,   0,-350,-100,   0,   0,-300,   0, 350, 100,   0}, // 00
#endif                                                                 ↑±逆↑       ↑±逆↑  ↑
  };
  int len = 20;
  double mt = 1;

  clear();

  for(int i(0); i< len;i++){
    RobotPosture *rp;
    if(i==0){
      rp = r->currentPosture->dupPosture();
    }else{
      rp = motion[i-1]->dupPosture();
    }
    rp->motionTime = (int)mt*100; /* sec -> msec */
    rp->setDegrees(pos_back[i]);
    motion.push_back( rp );
  }
  return;
}

void
RobotMotion::setLeft( SerialRobot *r )
{
#if 0
  int pos_left[4][20] ={ // かけ足（左）.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {    -50,-250,-550, 800, 1200,   0,   0, 1200,   0, -50, 350,-600, 300,   0,   0,   0, 350,-500, 300,  -50},
    {    -50,-600,-550, 800, 1200,   0,  10, 1200,   0, 200, 400,-550, 200,-200,   0, 350, 400,-550, 200, -300},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          ±逆       ±逆                        ←逆→                       ←逆→ */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {    -50,-250, 550, 800,-1200,   0,   0, 1200,   0, 350, -50,-600,-300,   0,   0,-350,   0, 500, 300,  -50},
    {    -50,-600, 550, 800,-1200,   0,  10, 1200,   0, 400, 200,-550,-200, 200,   0,-400,-350, 550, 200, -300},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif                                                                 ↑±逆↑       ↑±逆↑  ↑
  };
  int len = 4;
#else

  int pos_left[6][20] ={ // 左旋回.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {    -50,-400,-300, 400, 1300,-100,   0, 1200, 300,  50, 400,-500, 200,  50, 100,  50, 550,-800, 350, -100},
    {    -50,-550,-400, 200, 1200,-200,   0, 1200,   0,  50, 550,-800, 350,-100,-100,  50, 400,-500, 200,   50},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          ±逆       ±逆                        ←逆→                       ←逆→ */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {    -50,-400, 300, 400,-1300,-100,   0, 1200, 300, 400,  50,-500,-200, -50, 100,-550, -50, 800, 350, -100},
    {    -50,-550, 400, 200,-1200,-200,   0, 1200,   0, 550,  50,-800,-350, 100,-100,-400, -50, 500, 200,   50},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif                                                                 ↑±逆↑       ↑±逆↑  ↑
  };
  int len = 6;

#endif
  double mt = 0;

  clear();

  for(int i(0); i< len;i++){
    RobotPosture *rp;
    if(i==0){
      rp = r->currentPosture->dupPosture();
    }else{
      rp = motion[i-1]->dupPosture();
    }
    rp->motionTime = (int)mt*200; /* sec -> msec */
//    int joints = 20;
//    for(int j=0;j < joints;j++){
      rp->setDegrees(pos_left[i]);
//    }
    motion.push_back( rp );
    mt++;
  }
  return;
}

void
RobotMotion::setRight( SerialRobot *r )
{
  int pos_right[6][20] ={ // 右旋回.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {   -100, 700,-250,  50, 1300,-300, 200, 1300, 100,  20, 550,-800, 350, -70, 300,  80, 400,-500, 200,   20},
    {   -100, 500,-100,   0, 1300,-400, 200, 1200,-100, 120, 400,-500, 200, -20,   0,  80, 550,-800, 350, -130},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          ±逆       ±逆                        ←逆→                       ←逆→ */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {   -100, 700, 250,  50,-1300,-300, 200, 1300, 100, 550,  20,-800,-350,  70, 300,-400, -80, 500, 200,   20},
    {   -100, 500, 100,   0,-1300,-400, 200, 1200,-100, 400, 120,-500,-200,  20,   0,-550, -80, 800, 350, -130},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif                                                                 ↑±逆↑       ↑±逆↑  ↑
  };
  int len = 6;
  double mt = 0;

  clear();

  for(int i(0); i< len;i++){
    RobotPosture *rp;
    if(i==0){
      rp = r->currentPosture->dupPosture();
    }else{
      rp = motion[i-1]->dupPosture();
    }
    rp->motionTime = (int)mt*200; /* sec -> msec */
//    int joints = 20;
//    for(int j=0;j < joints;j++){
      rp->setDegrees(pos_right[i]);
//    }
    motion.push_back( rp );
    mt++;
  }
  return;
}

/********  Class Serial Robot *******************/
/**
  Constructor
*/

SerialRobot::SerialRobot(char *devname, int n): name("SerialRobot"),commandSize(0),timeout(500),motionTime(300),
executeMotion(false),repeatCount(1),reverseFlag(0),commandCount(0),senseTime(300),motionDir("")
{
  joints = n;
  hThread = NULL;
  hThread2 = NULL; // @@@
  threadLoop = 0;

#ifdef WIN32
  mutex_com = NULL;
  mutex_motion = NULL;
#endif

  com = new SerialCom(devname);

  initPosture = new RobotPosture(n);
  currentPosture = new RobotPosture(n);
  targetPosture =  new RobotPosture(n);

  motion = new RobotMotion();
  motion->setJoints(n);

  commandBuf = new char[128];
  servoState = new char[n];

  for(int i(0); i<n;i++){
    setServoState(i, 1);
  }

  jsf=H_NULL;
}

/**
  Deconstructor
*/
SerialRobot::~SerialRobot()
{
  stopThread();
  delete com;
  clearMotion();

  delete initPosture;
  delete currentPosture;
  delete targetPosture;
  delete commandBuf;
  delete servoState;
}


/*
   Open/Close the serial port.
*/
void
SerialRobot::setDevice(char *devname)
{
  com->setDevPort(devname);
}

int 
SerialRobot::openPort()
{
  return com->openPort();
}

int 
SerialRobot::connect()
{
  if(com->isConnected() > 0 ){ return 1; }
  if(openPort()  < 0){
    std::cerr << "Fail to open " << com->device <<std::endl;
      return -1;
  }
  if(checkConnection() < 0){
      closePort();
      return -1;
  }
   return 1;
}


void
SerialRobot::closePort()
{
  com->closePort();
  return;
}

/*
   Check connection?
*/
int 
SerialRobot::checkConnection()
{
  return 0;
}

/*

*/
int 
SerialRobot::numJoints()
{
  return joints;
}



/*
   Servo State (TEST)
*/
int 
SerialRobot::setServoState(int id, int state)
{
	if(id > joints){
     servoState[id-1] = state;
	}
   return state;
}

char *
SerialRobot::getServoState()
{
   return servoState;
}


int 
SerialRobot::setDefaultMotionTime(int sval)
{
  if(sval > 0 && sval < 10000){
     motionTime = sval;
  }else{
    std::cerr << "Invalid value = " <<  sval << "cs" << std::endl;
  }
  return motionTime;
}

/*


*/
int 
SerialRobot::getDefaultMotionTime(){
  return motionTime;
}


int 
SerialRobot::setTimeout(int val)
{
  timeout = val;
  return timeout;
}

int 
SerialRobot::getTimeout(){
  return timeout;
}

/*
   getAngle:
    This function should be overridden.
*/
short
SerialRobot::getAngle(unsigned char id)
{
  return -10000;
}

/*
   get all positions of joints.
*/
void 
SerialRobot::getPosture()
{
  int i;
  for(i = 1 ; i <= joints ; i++){
    if(getAngle((unsigned char)i) == -10000){
       LOCK_COM
	   if(com->clearBuffer() < 0){
		   com->closePort();
		   UNLOCK_COM
		   return;
	   }
       UNLOCK_COM
    }
  }
  return; 
}


/*
   Append a current posture
*/

int 
SerialRobot::appendCurrentPosture()
{
  motion->appendPosture(currentPosture->dupPosture());

  return motion->getSize();
}

int 
SerialRobot::appendCurrentPosture(double mtime)
{
  RobotPosture *pos = currentPosture->dupPosture();
  pos->setMotionTime(mtime);
  motion->appendPosture(pos);

 // motion->printMotion();

  return motion->getSize();
}

RobotPosture *
SerialRobot::getFirstPosture()
{
	if(motion->getSize() == 0) return NULL;
	return getNthPosture(0);
}

RobotPosture *
SerialRobot::getLastPosture()
{
	if(motion->getSize() == 0) return NULL;
	return getNthPosture(motion->getSize()-1);
}

RobotPosture *
SerialRobot::getNthPosture(int n)
{
	RobotPosture *pos;

	if(motion->getSize() < n){
		return NULL;
	}
	if(reverseFlag){
	  pos = motion->rget(n);
	}else{
	  pos = motion->get(n) ;
	}
	return pos;
}


RobotPosture *
SerialRobot::getTargetPosture()
{
	return targetPosture;
}

RobotPosture *
SerialRobot::getCurrentPosture()
{
	return currentPosture;
}



/*
   Load/Save the motion vector from file
*/
void 
SerialRobot::setMotionDir(const char *dir)
{
	motionDir = std::string(dir);
}

int
SerialRobot::loadMotion(char *name)
{
  int ret = -1;

  std::string filem(name);
  std::string fileyaml(name);
  std::string filepseq(name);
  std::string filemseq(name);

#if 0 // @@@
  filem += ".m";
  fileyaml += ".yaml";
  filepseq += ".pseq";
  filemseq += ".mseq";
#endif
  if(loadMotionFromM((char *)filem.c_str()) > 0){
    ret = motion->getSize();
  }else if(loadMotionFromYaml((char *)fileyaml.c_str()) > 0){
    ret = motion->getSize();
  }else if(loadMotionFromPseq((char *)filepseq.c_str()) > 0){
	ret = motion->getSize();
  }else if(loadMotionFromMseq((char *)filemseq.c_str()) > 0){
	ret = motion->getSize();
  }else{
	std::cout << "No such a motion, "<< name << std::endl;
  }
  return ret;
}

int
SerialRobot::loadMotionFromM(char *fname)
{
  std::string filename;

  if(motionDir == ""){
	  filename = fname;
  }else{
      filename = motionDir + FileDelim + fname;
  }
  clearMotion();

  if(!FileExists(filename.c_str(),"m")) { return -1; } // @@@

  filename += ".m"; // @@@
  motion->loadMotionFromFile(filename.c_str());

  return motion->getSize();
}

int
SerialRobot::loadMotionFromPseq(char *fname)
{
  std::string filename;

  if(motionDir == ""){
	  filename = fname;
  }else{
      filename = motionDir + FileDelim + fname;
  }
  clearMotion();

  if(!FileExists(filename.c_str(),"pseq")) { return -1; } // @@@

  filename += ".pseq"; // @@@
  motion->loadMotionFromPseqFile(filename.c_str(), this);
  return motion->getSize();
}

int
SerialRobot::loadMotionFromMseq(char *fname)
{
  std::string filename;

  if(motionDir == ""){
	  filename = fname;
  }else{
      filename = motionDir + FileDelim + fname;
  }
  clearMotion();

  if(!FileExists(filename.c_str(),"mseq")) { return -1; } // @@@

  filename += ".mseq"; // @@@
  motion->loadMotionFromMseqFile(filename.c_str(), this);
  return motion->getSize();
}

int
SerialRobot::loadMotionFromYaml(char *fname)
{
  std::string filename;

  if(motionDir == ""){
	  filename = fname;
  }else{
      filename = motionDir + FileDelim + fname;
  }
  clearMotion();

  if(!FileExists(filename.c_str(),"yaml")) { return -1; } // @@@

  filename += ".yaml"; // @@@
  motion->loadMotionFromYamlFile(filename.c_str(), this);
  return motion->getSize();
}


int
SerialRobot::saveMotionToM(char *fname)
{
  if(fname == NULL){
	motion->saveMotionToFile("Motion", motionDir.c_str());
  }else{
    std::string filename(motionDir);
    filename = motionDir + FileDelim + fname;

	motion->saveMotionToFile(filename.c_str());
  }

  return motion->getSize();
}

/*
  Not implemented...
*/
int
SerialRobot::saveMotionToPseq(char *fname)
{
  motion->saveMotionToPseqFile(fname, this);

  return motion->getSize();
}
/*
  Not implemented...
*/
int
SerialRobot::saveMotionToYaml(char *fname)
{
  motion->saveMotionToYamlFile(fname, this);

  return motion->getSize();
}


/***
   clear current motion sequence
**/
void 
SerialRobot::clearMotion()
{
  motion->clear();
}


/*
    Activate motion
*/
void 
SerialRobot::startMotion(){
  LOCK_MOTION
  commandCount = 1;
  UNLOCK_MOTION
}


int 
SerialRobot::doNthMotion(int n)
{
	RobotPosture *pos = getNthPosture(n);

	if(pos){
	  pos->setMotionTime(0); 

      targetPosture->copyPosture(pos);
      startMotion();
	}

  return 0;
}



int 
SerialRobot::setMotionCount(int count)
{
  if(motion == NULL || motion->motion.empty()){
	  return 0;
  }
  if(count < 0){
    repeatCount = -count;
    reverseFlag = 1;
    motion->setReverse(true);
  }else{
    repeatCount = count;
    reverseFlag = 0;
    motion->setReverse(false);
  }
  doNthMotion(0);
  LOCK_MOTION

  commandCount = motion->getSize() * repeatCount;
//std::cerr << "Cnt=" << commandCount << "Size = "<< motion->getSize() << std::endl; // @@@
  executeMotion = true;

  UNLOCK_MOTION

  return 1;
}


/*
    set an initial posture    
*/
int 
SerialRobot::initPosition()
{
	initPosture->printPosture();
  targetPosture->copyPosture(initPosture);
  startMotion();

  return 0;
}

int 
SerialRobot::setJoint( unsigned char id, short deg)
{
  targetPosture->setDegree(id, deg);
  return 0;
}

int 
SerialRobot::setMotionTime(int tm)
{
  targetPosture->setMotionTime((double)tm);
  return 0;
}

// @@@
int 
SerialRobot::selectMove( int n )
{
  clearMotion();

  switch(n) {
  case 1:
    motion->setForward(this);
    break;
  case 2:
    motion->setBack(this);
    break;
  case 3:
    motion->setLeft(this);
    break;
  case 4:
    motion->setRight(this);
    break;
  }
  return motion->getSize();
}


/**
  staet command thread
*/
int
SerialRobot::startThread()
{
  THREAD_FUNC thread_execution(void *args);

  Pthread_Mutex_Init(&mutex_com, NULL);
  Pthread_Mutex_Init(&mutex_motion, NULL);

  threadLoop = 1;
  if(Pthread_Create(&hThread, NULL, thread_execution, this) != 0){
    threadLoop = 0;
    return 0;
  }
  return 1;
}

int
SerialRobot::startThread2()
{
  THREAD_FUNC thread_chk_controller(void *args);

  jsf = open("/dev/input/js0", O_RDONLY);
  if(jsf < 0){
    std::cerr << "fail open startThread2" << std::endl;
    jsf = H_NULL;
    return -1;
  }

  if(Pthread_Create(&hThread2, NULL, thread_chk_controller, this) != 0){
    std::cerr << "fail create Thread2" << std::endl;
    threadLoop = 0;
    return 0;
  }
  return 1;
}

/**
 stop command thread
 */
int
SerialRobot::stopThread()
{
  if(threadLoop == 0) return 1;
  threadLoop = 0;

#ifdef WIN32
  if(hThread == NULL) return 1;
#else
  if(hThread == 0) return 1;
  if(hThread2== 0) return 1; // @@@
#endif

  Pthread_Join(hThread, NULL);
  Pthread_Mutex_Destroy(&mutex_com);
  Pthread_Mutex_Destroy(&mutex_motion);
  Pthread_Exit(hThread);
  Pthread_Join(hThread2, NULL); // @@@
  Pthread_Exit(hThread2); // @@@

#ifdef WIN32
  mutex_com = NULL;
  mutex_motion = NULL;
#endif

  hThread = NULL;
  hThread2 = NULL; // @@@
  close(jsf);
  return 0;
}

int 
SerialRobot::isActive()
{
  return threadLoop;
}



int 
SerialRobot::recieveData(char *data, int len){
  int res;
  LOCK_COM
  res = com->recieveData(data,len);
  UNLOCK_COM
  return res;
}
int
SerialRobot::isMoving()
{
	if(commandCount > 0){
		return 1;
	}else{
		return 0;
	}
}

int
SerialRobot::setCommand(char *packet, int len)
{
  commandSize=len;
  memcpy(commandBuf, packet, commandSize);
  return commandSize;
}


int 
SerialRobot::sendCommand(char *data, int len){
  int res = -1;

  if(this->connect() >0){

    LOCK_COM
#if 0 // @@@
    res = com->sendData(data,len);
#else
    if (data[0] == 0x53) { // @@@
      res = com->sendData(&data[2],(len-2)); // @@@
//      com->printPacket(&data[2],len-2); // @@@
    }
    else if (data[0] == 0x54) {
      res = com->sendData(&data[2],(len-3)); // @@@
//      com->printPacket(&data[2],len-3); // @@@
    }
#endif // @@@
    UNLOCK_COM
    if(res < 0){
      com->printPacket(data,len);
	  closePort();
    }

  }
  return res;
}

/**
   Background task to control robot.
*/
void
SerialRobot::svc()
{
  if(com->handle != H_NULL)
  {
    LOCK_MOTION

    if(commandCount > 0)
    {
      if(executeMotion)
      {
        targetPosture->copyPosture(motion->next());
      }
	  stabilizer();
      postureToCommand(targetPosture);
      sendCommand((char *)commandBuf, commandSize);

      if(executeMotion)
      {
        currentPosture->copyPosture(targetPosture);
      }
      commandCount -= 1;
      commandSize = 0;
    }else{
      timeout = senseTime;
      executeMotion = false;
      getPosture();
    }

    UNLOCK_MOTION

  }else{

    if(commandCount > 0)
    {
      if(executeMotion){
        targetPosture->copyPosture( motion->next() );
      }
	  stabilizer();
      postureToCommand(targetPosture);

      currentPosture->copyPosture(targetPosture);
      com->printPacket(commandBuf,commandSize);
     
      commandCount -= 1;
      commandSize = 0;
    }else{
      executeMotion = false;
      timeout = senseTime;
    }

  }
}

void
SerialRobot::svc2() // @@@
{
//  std::cerr << "svc2" << std::endl;
  if(jsf != H_NULL)
  {
    js_event js;
    read(jsf, &js, sizeof(js_event));

    switch(js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
      if (js.number < 4) {
//       std::cerr << "AXIS[" << js.number << "]: " << js.value << std::endl;
        printf("AXIS[%d]: %d  \n", js.number, js.value );
      }
      break;
    case JS_EVENT_BUTTON:
      switch(js.number) {
      case 0:
        if (js.value) { initPosition(); }
        break;
      case 4: // FORWORD
        if (js.value) { selectMove(1); setMotionCount(1); }
        break;
      case 5: // RIGHT
        if (js.value) { selectMove(4); setMotionCount(1); }
        break;
      case 6: // BACK
        if (js.value) { selectMove(2); setMotionCount(1); }
        break;
      case 7: // LEFT
        if (js.value) { selectMove(3); setMotionCount(1); }
        break;
      }
//       std::cerr << "BUTTON[" << js.number << "]: " << js.value << std::endl;
      printf("BUTTON[%d]: %d  \n", js.number, js.value );
      break;
    }
  }
}

/******** Thread Function *****/

#ifdef __cplusplus__
extern "C" {
#endif

short rad2deg(double d){
  short ret = (short)(d * Rad2Deg);
  return ret;
}

double deg2rad(double d){
  double ret = d * Deg2Rad ;
  return ret;
}

THREAD_FUNC thread_execution(void *args)
{

  int t1;
  int ms;

  SerialRobot *robot = (SerialRobot *)args;

  while(robot->isActive()){
    t1 = getCurrentTime();

    robot->svc();

    ms = getCurrentTime() - t1;
	//std::cerr << "Time = "<< ms << std::endl;
    ms = robot->getTimeout() - ms;

    if(ms > 0){
      Sleep(ms);
    }else{
      Sleep(0);
    }
  }
  std::cerr << "Thread terminated." << std::endl;

#ifdef WIN32
  return;
#else
  return NULL;
#endif
}

THREAD_FUNC thread_chk_controller(void *args)
{
  SerialRobot *robot = (SerialRobot *)args;

  while(robot->isActive()){
    robot->svc2();
  }
  std::cerr << "Thread terminated2." << std::endl;

#ifdef WIN32
  return;
#else
  return NULL;
#endif
}

#ifdef __cplusplus__
};
#endif
