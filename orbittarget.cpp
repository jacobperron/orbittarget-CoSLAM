#include "orbittarget.h"
#include "jacobutils.h"
#include <iostream>

//-----------------------------------------------------------------------------
COrbitTarget::COrbitTarget (ARobot* robot, std::string configFile, std::string logfile)
  : ARobotCtrl (robot) { 
  mName = robot->getName();
  mTime = 0.0;
  mText = 0;
  mVel = 0.0;
  mOmega = 0.0;
  mGo = 0;

  // Load parameters from config file
  //std::string configFile("orbittarget.conf"); 
  readConfigFile(configFile);
  
  mPrevYawError = 0.0;
  mPrevYawDError = 0.0;
  mPrevVError = 0.0;
  mPrevVDError = 0.0;
  mTargetFound = 0;

  mThreshYawError = D2R(7);
  mThreshNNAngleError = D2R(7);
  mThreshDistError = 0.2;
  mThreshNNDistError = 0.2;

  mRobot->findDevice(mDrivetrain, CB_DEVICE_DRIVE_TRAIN);
  mRobot->findDevice(mTextDisplay, CB_DEVICE_TEXT_DISPLAY);
  mRobot->findDevice(mIr, CB_DEVICE_IR);
#ifndef STAGE
  mRobot->findDevice(mLights, CB_DEVICE_LIGHTS);
  mRobot->findDevice(mWheelDrop, CB_DEVICE_WHEEL_DROP);

  // Establish connection to Redis server
  mRedis = new CRobotRedisClient(mName, HOSTNAME);
  if (mRedis == NULL) printf("Problem connecting to Redis server on %s\n", HOSTNAME);
#endif

  mDrivetrain = ( CCBDrivetrain2dof* ) mDrivetrain;
  mOdometry = mDrivetrain->getOdometry();

  // Enable logging
  mDataLogger = CDataLogger::getInstance(logfile, OVERWRITE);
  if (mDataLogger == NULL) printf("problem\n");
  mDataLogger->setInterval(0.1);
  mDataLogger->addVar(&mFrameTime, "time", 3);
  mDataLogger->addVar(&mTargetFound, "found");
  mDataLogger->addVar(&mPose, "follower");
  mDataLogger->addVar(&mDesiredYaw, "desired_yaw", 3);
  mDataLogger->addVar(&mTargetPose, "target");
  mDataLogger->addVar(&mTargetHeight, "target_height", 2);
  mDataLogger->addVar(&mVel, "v", 3);
  mDataLogger->addVar(&mOmega, "w", 3);
  mDataLogger->addVar(&mDesiredVel, "desired_v", 3);
  mDataLogger->addVar(&mDist, "dist", 3);
  mDataLogger->addVar(&mDesiredDist, "desired_dist", 3);
  mDataLogger->addVar(&mNNAngleDiff, "nn_angle_diff", 3);
  mDataLogger->addVar(&mNNAngleError, "nn_angle_error", 3);
  mDataLogger->addVar(&mNNDesiredDist, "nn_desired_dist", 3);
  mDataLogger->addVar(&mNNDistError, "nn_dist_error", 3);
  mDataLogger->addVar(&mNNDist, "nn_dist", 3);
  mDataLogger->addVar(&mVelError, "vel_error", 3);
  mDataLogger->addVar(&mYawError, "yaw_error", 3);
  mDataLogger->addVar(&mCircleYawError, "circle_yaw_error", 3);
  mDataLogger->addVar(&mDistError, "dist_error", 3);

  if ( rapiError->hasError() ) {
    rapiError->print();
    exit ( -1 );
  }

  // Set velocity and omega limits
  mVLimit.setLimit(-MAX_VEL, MAX_VEL);
  mWLimit.setLimit(-4, 4);
  printf("ready to go!\n");
}
//-----------------------------------------------------------------------------
COrbitTarget::~COrbitTarget() {
  printf("Deleting robot\n");
}
//-----------------------------------------------------------------------------
int COrbitTarget::readConfigFile(std::string& filePath){
  std::map<std::string, std::string> config = parse_config(filePath);
  setRobotSpeed(atof(config["speed"].c_str()));  
  setTargetHeight(atof(config["target_height"].c_str()));
  setDesiredRatio(atof(config["ratio"].c_str()));
  setDesiredAngle(atof(config["neighbour_angle"].c_str()));
  setWGains(atof(config["wp_gain"].c_str()),
            atof(config["wi_gain"].c_str()),
			atof(config["wd_gain"].c_str()));
  setVGains(atof(config["vp_gain"].c_str()),
            atof(config["vi_gain"].c_str()),
			atof(config["vd_gain"].c_str()));
  return 0;
}



//-----------------------------------------------------------------------------
void COrbitTarget::setDesiredRatio(double ratio) {
  mDesiredRatio = ratio;
  mDesiredPelHeight = mDesiredRatio * IMAGE_HEIGHT;
}
void COrbitTarget::setDesiredAngle(double a) {
  mDesiredNNAngle = normalizeAngle(D2R(a));
}
void COrbitTarget::setRobotSpeed(double speed) {
  VEL = speed;
  MAX_VEL = VEL + 0.1;
  MIN_VEL = VEL - 0.1;
}
void COrbitTarget::setTargetHeight(double height) {
  mTargetHeight = height;
}
void COrbitTarget::setVGains(double propGain, double integGain, double derivGain) {
  mVPGain = propGain;
  mVIGain = integGain;
  mVDGain = derivGain;
}
void COrbitTarget::setWGains(double propGain, double integGain, double derivGain) {
  mWPGain = propGain;
  mWIGain = integGain;
  mWDGain = derivGain;
}
void COrbitTarget::setNNName(std::string name) {
  mNNName = name;
}
#ifdef STAGE
void COrbitTarget::setStgTarget(Stg::Model* target) {
  stgTarget = target;
}
void COrbitTarget::setStgNN(Stg::Model* nn) {
  stgNN = nn;
}
#endif
//-----------------------------------------------------------------------------
void COrbitTarget::fetchFromRedis() {
#ifdef STAGE
  Stg::Pose tPose = stgTarget->GetGlobalPose();
  Stg::Pose nnPose = stgNN->GetGlobalPose();
  mPose = mOdometry->getPose();

  if (mTargetFound == 0) {
	mTargetFound = 1;
    mTargetPose.mX = tPose.x;
    mTargetPose.mY = tPose.y;
    mTargetPose.mYaw = tPose.a;
	mTargetPrevPose.mX = mTargetPose.mX;
	mTargetPrevPose.mY = mTargetPose.mY;
	mTargetPrevPose.mYaw = mTargetPose.mYaw;
  }
  else {
	mTargetPrevPose.mX = mTargetPose.mX;
	mTargetPrevPose.mY = mTargetPose.mY;
	mTargetPrevPose.mYaw = mTargetPose.mYaw;
    mTargetPose.mX = tPose.x;
    mTargetPose.mY = tPose.y;
    mTargetPose.mYaw = tPose.a;
  }

  mNNPose.mX = nnPose.x;
  mNNPose.mY = nnPose.y;
  mNNPose.mYaw = nnPose.a;
#else
  std::vector<std::string> info;
  info = mRedis->getMsg(mName);
  mFrameTime = atof(info[0].c_str());
  mTargetFound = atoi(info[1].c_str());
  mPose.mX = atof(info[3].c_str());
  mPose.mY = atof(info[4].c_str());
  mPose.mYaw = normalizeAngle(atof(info[5].c_str()));

  // Only update target info if currently being localized
  if (mTargetFound == 1) {
    std::vector<std::string> tInfo;
    tInfo = mRedis->getMsg("target");
    mTargetPrevPose.mX = mTargetPose.mX;
    mTargetPrevPose.mY = mTargetPose.mY;    
    mTargetPose.mX = atof(tInfo[1].c_str());
    mTargetPose.mY = atof(tInfo[2].c_str());
    //mTargetZ = atof(tInfo[3].c_str());
    //mTargetHeight = mTargetZ * 2.0;
  }
  
  std::vector<std::string> nnInfo;
  nnInfo = mRedis->getMsg(mNNName);
  mNNPose.mX = atof(nnInfo[3].c_str());
  mNNPose.mY = atof(nnInfo[4].c_str());
  mNNPose.mYaw = normalizeAngle(atof(nnInfo[5].c_str()));
#endif
}
//-----------------------------------------------------------------------------
void COrbitTarget::updateData (float dt)
{
  mTime += dt;
  mTimer += dt;

  // update info from Redis server
  fetchFromRedis();
  mDataLogger->write(mTime);

#ifndef STAGE
  // check for signal to move/stop
  std::string goSignal;
  mRedis->get("go", goSignal);
  if (goSignal == "g") mGo = 1;
  else mGo = 0;
#else
  mGo = 1;
#endif

  if (!emergencyStop()) {
    /** Use distance to target and desired image ratio to determine angular velocity **/
	// Use height of target and desired image ratio to determine desired distance to keep
	mDesiredDist = FOCAL_LENGTH * mTargetHeight / mDesiredPelHeight;

	// Compute angle to target
	double yawToTarget = normalizeAngle(atan2(mTargetPose.mY - mPose.mY, mTargetPose.mX - mPose.mX));
    double myYaw = normalizeAngle(mPose.mYaw);

	// Compute distance along camera's z-axis to target
    mDist = normalizeAngle(cos(myYaw + HALF_PI - yawToTarget)) * mPose.distance(mTargetPose);

	// Compute distance error (logging purposes only)
	mDistError = mDesiredDist - mDist;
	
    // Can want to head in two possible directions
    // 90 degrees, orbit target (tangent), or directly at target if too far away
    double circleYaw = normalizeAngle(yawToTarget - HALF_PI);

    // Compute ratio for desired yaw based on actual and desired distance to target
    double r = (-(mDesiredDist / mDist) + 1);
    //if (r > 1.0) r = 1.0;
    //if (r < 0.0) r = 0.0; 

	// Compute unit vectors for orbiting yaw and toward target yaw
	double c_x = cos(circleYaw);
	double c_y = sin(circleYaw);
	double tt_x = cos(yawToTarget);
	double tt_y = sin(yawToTarget);

	// Set desired yaw as weighted average of unit vectors
    mDesiredYaw = atan2(r*tt_y + (1-r)*c_y, r*tt_x + (1-r)*c_x);
	mDesiredYaw = normalizeAngle(mDesiredYaw);

    // Clamp desired yaw in order to keep target in view
    double upperYawBound = normalizeAngle(circleYaw + HALF_FOV);
	double lowerYawBound = normalizeAngle(circleYaw - HALF_FOV);
	double ub_x = cos(upperYawBound);
	double ub_y = sin(upperYawBound);
	double lb_x = cos(lowerYawBound);
	double lb_y = sin(lowerYawBound);
	double d_x = cos(mDesiredYaw);
	double d_y = sin(mDesiredYaw);

	// Compute dot products
	double dotUpper = ub_x*(-d_y) + ub_y*d_x;
	double dotLower = lb_x*(-d_y) + lb_y*d_x;

	// Check if to left of upperbound
	//(http://stackoverflow.com/questions/13221873/determining-if-one-2d-vector-is-to-the-right-or-left-of-another)
	if (dotUpper < 0) mDesiredYaw = upperYawBound;
	// Check if to right of lowerbound
	else if (dotLower > 0) mDesiredYaw = lowerYawBound;
	
	mDesiredYaw = normalizeAngle(mDesiredYaw);

    // Compute yaw error for feedback
    mYawError = normalizeAngle(mDesiredYaw - myYaw);
	// Error for log
	mCircleYawError = normalizeAngle(circleYaw - myYaw);

	// Compute turn rate
	double deltaYawError = mYawError - mPrevYawError;
    mOmega = mWPGain * mYawError + mWDGain * deltaYawError
	   						   + mWIGain * (deltaYawError - mPrevYawDError);
	mPrevYawDError = deltaYawError;
    mPrevYawError = mYawError;

	/** Use angle difference with nearest neighbour to throttle velocity */
	/** Use bearing to determine if neighbour is in front or behind in circle */
	// Compute angle difference
    //mNNAngleDiff = fabs(normalizeAngle(mPose.mYaw - mNNPose.mYaw));
	//mNNAngleError = normalizeAngle(mDesiredNNAngle - mNNAngleDiff);
	// Compute unit vector from NN to target
	double nnYawToTarget = normalizeAngle(atan2(mTargetPose.mY - mNNPose.mY, mTargetPose.mX - mNNPose.mX));
	double nn_x = cos(nnYawToTarget);
	double nn_y = sin(nnYawToTarget);

	// Dot product of NN unit vector and my unit vector to target
	double dotNN = nn_x*tt_x + nn_y*tt_y;
	mNNAngleDiff = normalizeAngle(acos(dotNN));
	mNNAngleError = normalizeAngle(mDesiredNNAngle - mNNAngleDiff);
	  
	// Compute bearing to neighbour
	double nnBearing = fabs(mPose.bearingTo(mNNPose));

	mNNDesiredDist = 2.0 * mDesiredDist * sin(mDesiredNNAngle / 2.0);
	mNNDist = mPose.distance(mNNPose);
	mNNDistError = mNNDesiredDist - mNNDist;

	// Determine the desired velocity, faster or slower
	double maxVel; // or min
	std::string nnLocale; // for debugging
	//if (mNNAngleError > 0) { // too close together
	if (mNNDistError > 0) {
	  if (nnBearing > HALF_PI) { // behind
	    maxVel = MAX_VEL;
	    nnLocale = "close-behind";
	  }
	  else { // in front
	    maxVel = MIN_VEL;
	    nnLocale = "close-front";
	  }
	}
	//else if (mNNAngleError < 0) { // too far apart
  	else if (mNNDistError < 0) {
	  if (nnBearing > HALF_PI) { // behind
	    maxVel = MIN_VEL;
	    nnLocale = "far-behind";
	  }
	  else { // in front
	    maxVel = MAX_VEL;
	    nnLocale = "far-front";
	  }
	}
	else {
	  maxVel = VEL;
	  nnLocale = "perfect"; // far from likely
	}

	double r2 = fabs(mNNAngleError / QUARTER_PI);
	//if (r2 > 1.0) r2 = 1.0;
	//if (r2 < 0.0) r2 = 0.0;
	mDesiredVel = r2 * maxVel + (1-r2) * VEL;
	// Compute velocity error
	mVelError = mDesiredVel - mVel; //maxVel - mVel; // fabs?

	// Compute change in velocity
	double deltaVError = mVelError - mPrevVError;
	// PID to desired velocity
	mVel += mVPGain * mVelError + mVDGain * deltaVError
		  						  + mVIGain * (deltaVError - mPrevVDError);

	mPrevVDError = deltaVError;
	mPrevVError = mVelError;

	
	// Check for stopping condition, if all sources of error are at an acceptable level
	/*if (fabs(mYawError) < mThreshYawError &&
	    fabs(mNNAngleError) < mThreshNNAngleError &&
        fabs(mDistError) < mThreshDistError) mGo = 0;
	else
	  mGo = 1;
    */

	// state info for debug
	//std::cout << mName << " qerror: " << myYaw - circleYaw << "\tderror: " << mDesiredDist - mDist;
	//std::cout << "\tnerror: " << mNNDistError << std::endl;
	

	//std::cout << "desiredDist: " << mDesiredDist << "\tdist: " << mDist << std::endl;
	//printf("v: %.2f\tw: %.0f\tdesired_yaw: %.0f\tyaw: %.0f\tNNangleDiff: %.0f\tmYawError: %.1f\tmNNAngleError: %.1f\t%s\tpose: %.1f,%.1f,%.1f\ttpose: %.1f,%.1f,%.1f\n", mVel, R2D(mOmega), R2D(mDesiredYaw), R2D(myYaw), R2D(mNNAngleDiff), R2D(mYawError), R2D(mNNAngleError), nnLocale.c_str(), mPose.mX, mPose.mY, R2D(mPose.mYaw), mTargetPose.mX, mTargetPose.mY, mTargetZ);
	//std::cout << "yawtoTarget: " << R2D(yawToTarget) << std::endl;

	// Limit velocities
	mVel = mVLimit.limit(mVel);
	//mOmega = mWLimit.limit(mOmega);

	// Drive
	mDrivetrain->setVelocityCmd(mGo * mVel, mGo * mOmega);
  }
  else {
    PRT_MSG0(4, "Emergency stop.");
  }

  if ( rapiError->hasError() ) {
    rapiError->print();
  }
}
//-----------------------------------------------------------------------------
bool COrbitTarget::emergencyStop()
{
  bool stop = false;
#ifndef STAGE
  // Check for wheel drop
  if ( mWheelDrop->isAnyTriggered() ) {
    stop = true;
  }

  // Check for cliffs?

  // set lights red and drivetrain to stop if stop condition detected
  if (stop) {
    mDrivetrain->stop();
    mLights->setLight ( ALL_LIGHTS, RED );
  }
  else {
    mLights->setLight ( ALL_LIGHTS, BLACK );
  }
#endif
  return stop;
}
//-----------------------------------------------------------------------------

