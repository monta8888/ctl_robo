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
	|| !topMap->equalTo((char*)"type", (char*)"MotionSeq")
	|| (refs = topMap->getSequence((char*)"refs")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }

  clear();

  int len = refs->size();
  double st = 0;
  double mt;
//  short deg = 0;

  for(int i(0); i< len;i++){
    RobotPosture *rp;

	YamlMapping *ref_i = refs->getMappingAt(i);

	if(ref_i == NULL || ref_i->getScalar((char*)"time") == NULL)
	{
		continue;
	}
 
	mt = ref_i->getScalar((char*)"time")->toInteger();

	if(i==0){
      //  rp = new RobotPosture(numJoints);
		rp = r->currentPosture->dupPosture();
	}else{
		rp = motion[i-1]->dupPosture();
	}

    rp->motionTime = (int)((mt - st)*1000); /* sec -> msec */
    st = mt;

	YamlSequence *joint_seq, *q_seq;

	if( ref_i->getMapping((char*)"refer") == NULL || 
		(joint_seq = ref_i->getMapping((char*)"refer")->getSequence((char*)"joints")) == NULL ||
		(q_seq = ref_i->getMapping((char*)"refer")->getSequence((char*)"q")) == NULL )
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
	|| !topMap->equalTo((char*)"type", (char*)"PoseSeq")
	|| (refs = topMap->getSequence((char*)"refs")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }

  clear();

  int len = refs->size();
  double st = 0;
  double mt;
//  short deg = 0;

  for(int i(0); i< len;i++){
    RobotPosture *rp;

	YamlMapping *ref_i = refs->getMappingAt(i);

	if(ref_i == NULL || ref_i->getScalar((char*)"time") == NULL)
	{
		continue;
	}
 
	mt = ref_i->getScalar((char*)"time")->toInteger();

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

	if( ref_i->getMapping((char*)"refer") == NULL || 
		(joint_seq = ref_i->getMapping((char*)"refer")->getSequence((char*)"joints")) == NULL ||
		(q_seq = ref_i->getMapping((char*)"refer")->getSequence((char*)"q")) == NULL )
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
	|| !topMap->equalTo((char*)"type", (char*)"BodyMotion")
	|| (comps = topMap->getSequence((char*)"components")) == NULL )
  {
    std::cerr << "Invalid Yaml format : " << name << std::endl;
    return false; 
  }
 
  clear();

  for( int i(0); i<comps->size() ;i++){
	YamlMapping *m = comps->at(i)->toMapping();

	if(m == NULL) continue;
	if( m->equalTo((char*)"type", (char*)"MultiValueSeq") && 
			m->equalTo((char*)"purpose", (char*)"JointPosition") )
	{
      int frate, nFrames;

	  if(m->getScalar((char*)"frameRate") == NULL ){
		 frate = 10;
	  }else{
         frate = m->getScalar((char*)"frameRate")->toInteger();
	  }
	  if(m->getScalar((char*)"numFrames") == NULL){
        nFrames = 0;
	  }else{
		nFrames = m->getScalar((char*)"numFrames")->toInteger();
	  }

      double tm = 1000.0 / frate;
//      short deg=0;

      frames = m->getSequence((char*)"frames");
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

bool 
RobotMotion::loadMotionFromMtnFile(const char *name, SerialRobot *r)
{
  std::ifstream ifs;
  ifs.open(name);

  if( !ifs.fail() ){
    clear();
//    double st = 0;
    char line[1024];

    if( !ifs.eof() ){
      ifs.getline(line,1024);
      std::istringstream iss(line);
//      iss.str(line.substr(16,17));
      char str[10];
      int num;
      iss >> str;
      iss >> num;
std::cerr << "HEAD[" << str << ":" << num << std::endl;
    }
    if( !ifs.eof() ){
      ifs.getline(line,1024);
      std::istringstream iss(line);
      int num;
      iss >> num;
std::cerr << "POSNUM[" << num << std::endl;
    }

    if( !ifs.eof() ){
      ifs.getline(line,1024);
      ifs.getline(line,1024);
      std::istringstream iss(line);
      char str[4];
      int num;
      iss >> str;
      iss >> num;
std::cerr << "KKKK[" << str << ":" << num << std::endl;
    }
    if( !ifs.eof() ){
      ifs.getline(line,1024);
      std::istringstream iss(&line[4]);
//      iss.str(buffer.substr(4,1024));
      char str[1024];
      iss >> str;
std::cerr << "PPPP[" << str << std::endl;
    }

    if( !ifs.eof() ){
      ifs.getline(line,1024);
      ifs.getline(line,1024);
      std::istringstream iss(line);
      char str[4];
      int num;
      iss >> str;
      iss >> num;
std::cerr << "KKKK[" << str << ":" << num << std::endl;
    }
    if( !ifs.eof() ){
      ifs.getline(line,1024);
      std::istringstream iss(&line[4]);
//      iss.str(buffer.substr(4,1024));
      char str[1024];
      iss >> str;
std::cerr << "PPPP[" << str << std::endl;
    }
#if 0
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
#endif
    ifs.close();
    return true;
  }else{
    std::cerr << "Faile to open " << name << std::endl;
  }

  return false;
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
  S01çò
  S02ì™
  S03âEå®
  S04âEìÒÇÃòr
  S05âEòr
  S06ç∂å®
  S07ç∂ìÒÇÃòr
  S08ç∂òr
  S09âEïtÇØç™
  S10âEëæÇ‡Ç‡ëO
  S11âEëæÇ‡Ç‡å„
  S12âEïG
  S13âEë´éÒëO
  S14âEë´éÒå„
  S15ç∂ïtÇØç™
  S16ç∂ëæÇ‡Ç‡ëO
  S17ç∂ëæÇ‡Ç‡å„
  S18ç∂ïG
  S19ç∂ë´éÒëO
  S20ç∂ë´éÒå„
 */
// @@@
void
RobotMotion::setForward( SerialRobot *r )
{
  int pos_forward[20][20] ={ // ëOêi.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0,-250,  50,  500,-250,  50,  500,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200,-100}, // Soc
    {      0,   0,-250,   0, 1200,-250,   0, 1200,   0,   0, 350,-450, 150,   0,   0,   0, 350,-450, 150,   0}, // 00
    {      0,   0,-400,   0, 1200, -50,   0, 1200,   0,  50, 450,-650, 280,  20,   0,   0, 200,-350, 200,  20}, // 01
    {      0,   0,-250,   0, 1200,-250,   0, 1300,   0,   0, 400,-350,   0,   0,   0,   0, 200,-350, 200,   0}, // 03
    {      0,   0, -50,   0, 1200, -50,   0, 1300,   0,   0, 200,-350, 200,  20,   0,  50, 450,-650, 220,  40}, // 02
    {      0,   0,-250,   0, 1200,-250,   0, 1300,   0,   0, 200,-350, 200,   0,   0,   0, 400,-350,   0,   0}, // 04
    {      0,   0,-250,   0, 1200,-250,   0, 1200,   0,   0, 350,-450, 150,   0,   0,   0, 350,-450, 150,   0}, // 00
#else /*          Å}ãt       Å}ãt                        Å©ãtÅ®                       Å©ãtÅ® */
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
#endif /*                                                              Å™Å}ãtÅ™       Å™Å}ãtÅ™  Å™ */
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
  int pos_back[20][20] ={ // å„ëﬁ.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0,-250,  50,  500,-250,  50,  500,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200,-100}, // Soc
    {      0,   0, 100,   0, 1200, 100,   0, 1200,   0,   0, 300,-350, 100,   0,   0,   0, 300,-350, 100,   0}, // 00
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,  50, 400,-650, 280, -20,   0,   0, 350,-350,  50,  20}, // 01
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 250,-350, 150,   0,   0,   0, 350,-350,  50,   0}, // 03
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 350,-350,  50,  20,   0,  50, 450,-650, 220,  40}, // 02
    {      0,   0,   0,   0, 1200,   0,   0, 1200,   0,   0, 350,-350,  50,   0,   0,   0, 250,-350, 150,   0}, // 04
    {      0,   0, 100,   0, 1200, 100,   0, 1200,   0,   0, 300,-350, 100,   0,   0,   0, 300,-350, 100,   0}, // 00
#else /*          Å}ãt       Å}ãt                        Å©ãtÅ®                       Å©ãtÅ® */
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
#endif /*                                                              Å™Å}ãtÅ™       Å™Å}ãtÅ™  Å™ */
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
  int pos_left[4][20] ={ // Ç©ÇØë´Åiç∂Åj.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {    -50,-250,-550, 800, 1200,   0,   0, 1200,   0, -50, 350,-600, 300,   0,   0,   0, 350,-500, 300,  -50},
    {    -50,-600,-550, 800, 1200,   0,  10, 1200,   0, 200, 400,-550, 200,-200,   0, 350, 400,-550, 200, -300},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          Å}ãt       Å}ãt                        Å©ãtÅ®                       Å©ãtÅ® */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {    -50,-250, 550, 800,-1200,   0,   0, 1200,   0, 350, -50,-600,-300,   0,   0,-350,   0, 500, 300,  -50},
    {    -50,-600, 550, 800,-1200,   0,  10, 1200,   0, 400, 200,-550,-200, 200,   0,-400,-350, 550, 200, -300},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif                                                                 Å™Å}ãtÅ™       Å™Å}ãtÅ™  Å™
  };
  int len = 4;
#else

  int pos_left[6][20] ={ // ç∂ê˘âÒ.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {    -50,-400,-300, 400, 1300,-100,   0, 1200, 300,  50, 400,-500, 200,  50, 100,  50, 550,-800, 350, -100},
    {    -50,-550,-400, 200, 1200,-200,   0, 1200,   0,  50, 550,-800, 350,-100,-100,  50, 400,-500, 200,   50},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          Å}ãt       Å}ãt                        Å©ãtÅ®                       Å©ãtÅ® */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {    -50,-400, 300, 400,-1300,-100,   0, 1200, 300, 400,  50,-500,-200, -50, 100,-550, -50, 800, 350, -100},
    {    -50,-550, 400, 200,-1200,-200,   0, 1200,   0, 550,  50,-800,-350, 100,-100,-400, -50, 500, 200,   50},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif /*                                                              Å™Å}ãtÅ™       Å™Å}ãtÅ™  Å™ */
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
  int pos_right[6][20] ={ // âEê˘âÒ.mtn
#if 0 /* S01  S02  S03  S04   S05  S06  S07   S08  S09  S10  S11  S12  S13  S14  S15  S16  S17  S18  S19  S20 */
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {   -100, 700,-250,  50, 1300,-300, 200, 1300, 100,  20, 550,-800, 350, -70, 300,  80, 400,-500, 200,   20},
    {   -100, 500,-100,   0, 1300,-400, 200, 1200,-100, 120, 400,-500, 200, -20,   0,  80, 550,-800, 350, -130},
    {   -100,   0, 500, 200, 1200, 500, 200, 1200,   0,   0, 400,-550, 200,   0,   0,   0, 400,-550, 200,   50},
    {      0,   0, 250,-100, 1200, 250,-100, 1200,   0, 100, 400,-550, 200,-100,   0, 100, 400,-550, 200, -100},
#else /*          Å}ãt       Å}ãt                        Å©ãtÅ®                       Å©ãtÅ® */
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {   -100, 700, 250,  50,-1300,-300, 200, 1300, 100, 550,  20,-800,-350,  70, 300,-400, -80, 500, 200,   20},
    {   -100, 500, 100,   0,-1300,-400, 200, 1200,-100, 400, 120,-500,-200,  20,   0,-550, -80, 800, 350, -130},
    {   -100,   0,-500, 200,-1200, 500, 200, 1200,   0, 400,   0,-550,-200,   0,   0,-400,   0, 550, 200,   50},
    {      0,   0,-250,-100,-1200, 250,-100, 1200,   0, 400, 100,-550,-200, 100,   0,-400,-100, 550, 200, -100},
#endif /*                                                              Å™Å}ãtÅ™       Å™Å}ãtÅ™  Å™ */
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

SerialRobot::SerialRobot(char *devname, int brate, int n): name((char*)"SerialRobot"),motionDir((char*)""),motionTime(300),senseTime(300),timeout(500),
repeatCount(1),reverseFlag(0),executeMotion(false),commandCount(0),commandSize(0)
{
  joints = n;
  hThread  = 0;
  hThread2 = 0; // @@@
  threadLoop = 0;

#ifdef WIN32
  mutex_com = NULL;
  mutex_motion = NULL;
#endif

  com = new SerialCom(devname, brate);

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
#ifdef DEBUG_PRINT
      std::cerr << "Fail to open " << com->device <<std::endl;
#endif
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
#ifdef DEBUG_PRINT
    std::cerr << "Invalid value = " <<  sval << "cs" << std::endl;
#endif
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
  std::string filemtn(name); // @@@

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
  }else if(loadMotionFromMtn((char *)filemseq.c_str()) > 0){ // @@@
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

int // @@@
SerialRobot::loadMotionFromMtn(char *fname)
{
  std::string filename;

  if(motionDir == ""){
	  filename = fname;
  }else{
      filename = motionDir + FileDelim + fname;
  }
  clearMotion();

  if(!FileExists(filename.c_str(),"mtn")) { return -1; } // @@@

  filename += ".mtn";
  motion->loadMotionFromMtnFile(filename.c_str(), this);
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
#ifdef DEBUG_PRINT
  initPosture->printPosture();
#endif
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
#ifdef CTL_8GPIO
void
SerialRobot::set_Noled()
{
  mraa_gpio_write(gpio45_g, 0); //
  mraa_gpio_write(gpio46_f, 0); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 0); //
  mraa_gpio_write(gpio49_a, 0); //
  mraa_gpio_write(gpio50_b, 0); //
  mraa_gpio_write(gpio54_c, 0); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_Dpled()
{
  mraa_gpio_write(gpio45_g, 0); //
  mraa_gpio_write(gpio46_f, 0); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 0); //
  mraa_gpio_write(gpio49_a, 0); //
  mraa_gpio_write(gpio50_b, 0); //
  mraa_gpio_write(gpio54_c, 0); //
  mraa_gpio_write(gpio55_dp,1); //
}

void
SerialRobot::set_0led()
{
  mraa_gpio_write(gpio45_g, 0); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 1); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,1); //
}

void
SerialRobot::set_1led()
{
  mraa_gpio_write(gpio45_g, 0); //
  mraa_gpio_write(gpio46_f, 0); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 0); //
  mraa_gpio_write(gpio49_a, 0); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_2led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 0); //
  mraa_gpio_write(gpio47_e, 1); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 0); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_3led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 0); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_4led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 0); //
  mraa_gpio_write(gpio49_a, 0); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_5led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 0); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_6led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 1); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 0); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_7led()
{
  mraa_gpio_write(gpio45_g, 0); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 0); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_8led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 1); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}

void
SerialRobot::set_9led()
{
  mraa_gpio_write(gpio45_g, 1); //
  mraa_gpio_write(gpio46_f, 1); //
  mraa_gpio_write(gpio47_e, 0); //
  mraa_gpio_write(gpio48_d, 1); //
  mraa_gpio_write(gpio49_a, 1); //
  mraa_gpio_write(gpio50_b, 1); //
  mraa_gpio_write(gpio54_c, 1); //
  mraa_gpio_write(gpio55_dp,0); //
}
#endif

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

  if(Pthread_Create(&hThread2, NULL, thread_chk_controller, this) != 0){
#ifdef DEBUG_PRINT
    std::cerr << "fail create Thread2" << std::endl;
#endif
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

#ifdef CTL_7SEG
  mraa_gpio_close(gpio20_D3);
  mraa_gpio_close(gpio21_D0);
  mraa_gpio_close(gpio33_D1);
  mraa_gpio_close(gpio36_D2);
  mraa_deinit();
#elif defined(CTL_8GPIO)
  mraa_gpio_close(gpio45_g);
  mraa_gpio_close(gpio46_f);
  mraa_gpio_close(gpio47_e);
  mraa_gpio_close(gpio48_d);
  mraa_gpio_close(gpio49_a);
  mraa_gpio_close(gpio50_b);
  mraa_gpio_close(gpio54_c);
  mraa_gpio_close(gpio55_dp);
  mraa_deinit();
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

  hThread  = 0;
  hThread2 = 0; // @@@
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

int
SerialRobot::svc2(int *cnt, int *stat) // @@@
{
//  std::cerr << "svc2" << std::endl;
  if (!*stat) {
#ifdef CTL_7SEG
    mraa_init();
    gpio20_D3 = mraa_gpio_init(20); // GPIO 12/J18-7
    mraa_gpio_use_mmaped(gpio20_D3, 1); // FastIO
    mraa_gpio_dir(gpio20_D3, MRAA_GPIO_OUT);
    gpio21_D0 = mraa_gpio_init(21); // GPIO183/J18-8
    mraa_gpio_use_mmaped(gpio21_D0, 1); // FastIO
    mraa_gpio_dir(gpio21_D0, MRAA_GPIO_OUT);
    gpio33_D1 = mraa_gpio_init(33); // GPIO 48/J19-6
    mraa_gpio_use_mmaped(gpio33_D1, 1); // FastIO
    mraa_gpio_dir(gpio33_D1, MRAA_GPIO_OUT);
    gpio36_D2 = mraa_gpio_init(36); // GPIO 14/J19-9
    mraa_gpio_use_mmaped(gpio36_D2, 1); // FastIO
    mraa_gpio_dir(gpio36_D2, MRAA_GPIO_OUT);
#elif defined(CTL_8GPIO)
    mraa_init();
    gpio45_g = mraa_gpio_init(45); // GPIO 45/J20-4
    mraa_gpio_use_mmaped(gpio45_g, 1); // FastIO
    mraa_gpio_dir(gpio45_g, MRAA_GPIO_OUT);
    gpio46_f = mraa_gpio_init(46); // GPIO 46/J20-5
    mraa_gpio_use_mmaped(gpio46_f, 1); // FastIO
    mraa_gpio_dir(gpio46_f, MRAA_GPIO_OUT);
    gpio47_e = mraa_gpio_init(47); // GPIO 47/J20-6
    mraa_gpio_use_mmaped(gpio47_e, 1); // FastIO
    mraa_gpio_dir(gpio47_e, MRAA_GPIO_OUT);
    gpio48_d = mraa_gpio_init(48); // GPIO 48/J20-7
    mraa_gpio_use_mmaped(gpio48_d, 1); // FastIO
    mraa_gpio_dir(gpio48_d, MRAA_GPIO_OUT);
    gpio49_a = mraa_gpio_init(49); // GPIO 49/J20-8
    mraa_gpio_use_mmaped(gpio49_a, 1); // FastIO
    mraa_gpio_dir(gpio49_a, MRAA_GPIO_OUT);
    gpio50_b = mraa_gpio_init(50); // GPIO 50/J20-9
    mraa_gpio_use_mmaped(gpio50_b, 1); // FastIO
    mraa_gpio_dir(gpio50_b, MRAA_GPIO_OUT);
    gpio54_c = mraa_gpio_init(54); // GPIO 54/J20-13
    mraa_gpio_use_mmaped(gpio54_c, 1); // FastIO
    mraa_gpio_dir(gpio54_c, MRAA_GPIO_OUT);
    gpio55_dp= mraa_gpio_init(55); // GPIO 55/J20-14
    mraa_gpio_use_mmaped(gpio55_dp, 1); // FastIO
    mraa_gpio_dir(gpio55_dp, MRAA_GPIO_OUT);
#endif
    *stat = 1;
  }
  if(jsf == H_NULL) {
    jsf = open("/dev/input/js0", O_RDONLY);
    if(jsf < 0){
//      std::cerr << "fail open startThread2" << std::endl;
      jsf = H_NULL;
//      std::cerr << "LED Counter" << *cnt << std::endl;
#ifdef CTL_7SEG
      mraa_gpio_write(gpio20_D3, (*cnt&1));
      mraa_gpio_write(gpio36_D2, (*cnt&1));
      mraa_gpio_write(gpio33_D1, (*cnt&1));
      mraa_gpio_write(gpio21_D0, (*cnt&1));
#elif defined(CTL_8GPIO)
#if 0 // test
  switch(*cnt&0xf) {
  case 0: set_0led(); break;
  case 1: set_1led(); break;
  case 2: set_2led(); break;
  case 3: set_3led(); break;
  case 4: set_4led(); break;
  case 5: set_5led(); break;
  case 6: set_6led(); break;
  case 7: set_7led(); break;
  case 8: set_8led(); break;
  case 9: set_9led(); break;
  }
#endif
      if (*cnt&1) { set_Dpled(); } else { set_Noled(); }
#endif
      *cnt = *cnt + 1;
      return -1;
    }
    else {
#ifdef CTL_7SEG
      mraa_gpio_write(gpio20_D3, 0);
      mraa_gpio_write(gpio36_D2, 0);
      mraa_gpio_write(gpio33_D1, 0);
      mraa_gpio_write(gpio21_D0, 1);
#elif defined(CTL_8GPIO)
      set_0led();
#endif
    }
  }

  if(jsf != H_NULL)
  {
    js_event js;
    read(jsf, &js, sizeof(js_event));

    switch(js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
      if (js.number < 4) {
//       std::cerr << "AXIS[" << js.number << "]: " << js.value << std::endl;
#ifdef DEBUG_PRINT
        printf("AXIS[%d]: %d  \n", js.number, js.value );
#endif
      }
      break;
    case JS_EVENT_BUTTON:
//       std::cerr << "BUTTON[" << js.number << "]: " << js.value << std::endl;
#ifdef DEBUG_PRINT
      printf("BUTTON[%d]: %d  \n", js.number, js.value );
#endif
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
//      printf("BUTTON fin\n");
      break;
    }
  }
  return 0;
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
#ifdef DEBUG_PRINT
  std::cerr << "Thread terminated." << std::endl;
#endif
#ifdef WIN32
  return;
#else
  return NULL;
#endif
}

THREAD_FUNC thread_chk_controller(void *args)
{
  int led_cnt =0;
  int led_stat=0;
  SerialRobot *robot = (SerialRobot *)args;

  while(robot->isActive()){
    if (robot->svc2(&led_cnt, &led_stat)) {
      Sleep(500);
    }
  }
#ifdef DEBUG_PRINT
  std::cerr << "Thread terminated2." << std::endl;
#endif
#ifdef WIN32
  return;
#else
  return NULL;
#endif
}

#ifdef __cplusplus__
};
#endif
