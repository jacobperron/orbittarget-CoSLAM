/***************************************************************************
 * Project: CO-SLAM Target Following                                       *
 * Author:  Jacob Perron (jperron@sfu.ca)                                  *
 ***************************************************************************
 * A controller for a Chatterbox robot to orbit a moving target.           *
 * Includes Stage simulation support                                       *
 * Assumes given pose information about robot and target                   *
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/
#ifndef ORBITTARGET_H
#define ORBITTARGET_H

#include "RapiChatterbox"
#ifdef STAGE
#include "RapiStage"
#endif

using namespace Rapi;

// Redis server hostname (where pose information is published)
#define HOSTNAME "192.168.1.137"

// Camera properties
const double FOCAL_LENGTH = 643.95;
const double IMAGE_HEIGHT = 480;
const double HALF_FOV = D2R(25);

/**
 * A RAPI controller for Chatterbox that orbits a moving target using CoSLAM.
 * @author Jacob Perron
 */
class COrbitTarget : public ARobotCtrl
{
  public:
    /**
     * Default constructor
     * @param robot : the robot this controller controls
     * @param logfile : the name of the log file to write to
     */
    COrbitTarget(ARobot* robot,
			     std::string configFile = "orbittarget.conf",
				 std::string logfile = "orbittarget.log");

    /** Default destructor */
    ~COrbitTarget();

    /**
     * Set the gains for the PID controller of velocity.
     */
    void setVGains(double propGain, double derivGain, double integGain);
    
	/**
     * Set the gains for the PID controller of angular velocity.
     */
    void setWGains(double propGain, double derivGain, double integGain);

	/**
	 * Set the desired ratio of the target in the camera image
	 */
	void setDesiredRatio(double ratio);

	/**
	 * Set robot's speed.
	 */
	void setRobotSpeed(double speed);
	
	/**
	 * Set the name of the nearest neighbouring robot.
	 */
	void setNNName(std::string name);

	/**
	 * Set target's height.
	 */
	void setTargetHeight(double height);

	/**
	 * Set the desired angle difference with the nearest neighbour
	 */
	void setDesiredAngle(double angle);
#ifdef STAGE
	/**
	 * Set Stage model of target
	 */
	void setStgTarget(Stg::Model* target);

	/**
	 * Set Stage model of nearest neighbour
	 */
	void setStgNN(Stg::Model* nn);
#endif
  protected:
    /**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);

    /** Emergency stop routine */
    bool emergencyStop();

	/** 
	 * Retrieve information provided by CoSLAM running on base station.
	 */
	void fetchFromRedis();

	/** Drivetrain */
    ADrivetrain2dof* mDrivetrain;
    /** Text display */
    ATextDisplay* mTextDisplay;
    /** IR sensor */
    ARangeFinder* mIr;
    /** Odometry */
    COdometry* mOdometry;
	/** Lights */
    ALights* mLights;
    /** Bumper */
    //ABinarySensorArray* mBumper;
    /** Wheel drop */
    ABinarySensorArray* mWheelDrop;
    /** Low side driver */
    //ASwitchArray* mLowSideDriver;
    /** Top fiducial */
    //AFiducialFinder* mTopFiducial;
    /** Front fiducial */
    //AFiducialFinder* mFrontFiducial;
    /** Photo sensor */
    //ABinarySensorArray* mButton;
    /** Cliff sensor */
    //ABinarySensorArray* mCliff;
    /** Limits for velocity and omega */
    CLimit mVLimit;
    CLimit mWLimit;
	/** Data Logger, ...duh */
	CDataLogger* mDataLogger;

  private:
	/** Name of robot */
	std::string mName;
	/** Velocity of robot */
	double mVel;
	/** Angular velocity of robot */
	double mOmega;
	/** Distance from target */
	double mDist;
	
	/** Desired distance to target */
	double mDesiredDist;
	/** Desired yaw */
	double mDesiredYaw;
	/** Desired velocity */
	double mDesiredVel;
	/** Desried distance to nearest neighbour */
	double mNNDesiredDist;
    /** Desired ratio of target height in camera image */
	double mDesiredRatio;
	/** Desired angle to keep with heading of nn */
	double mDesiredNNAngle;
	/** Desired height in pixels (derived from desired ratio) */
	double mDesiredPelHeight;
	
	/** Name of nearest neighbour */
	std::string mNNName;
	/** Angle difference with nn */
	double mNNAngleDiff;
	/** Distance to nn */
	double mNNDist;

	/** Distance to target error */
	double mDistError;
	/** Current yaw error wrt desired yaw*/
	double mYawError;
    /** Previous yaw error to compute derivative */
    double mPrevYawError;
	/** Previous derivative error */
	double mPrevYawDError;
	/** Yaw error wrt circle yaw */
	double mCircleYawError;
	/** Current nn angle error */
	double mNNAngleError;
	/** Error between desired and actual distance to nn */
	double mNNDistError;
	/** Current velocity error */
	double mVelError;
	/** Previous velocty error */
    double mPrevVError;
	double mPrevVDError;

	/** Proportional gain for velocity PD controller */
    double mVPGain;
    /** Derivative gain for veloctiy PD controller */
    double mVDGain;
    /** Integral gain for veloctiy PD controller */
    double mVIGain;
	/** Proportional gain for angular velocity controller */
	double mWPGain;
	/** Derivative gain for angular velocity controller */
	double mWDGain;
	/** Integral gain for angular velocity controller */
	double mWIGain;

	/** Time since start of controller [s] */
    double mTime;
    /** Text for 7seg display */
    int mText;
    /** General purpose timer */
    double mTimer;
    /** Move flag. Set to 1 to move, set to 0 to stop */
	int mGo;
	
	/** Thresholds for stopping condition */
	double mThreshYawError;
	double mThreshNNAngleError;
	double mThreshDistError;
	double mThreshNNDistError;

	/** Redis client for communication with CoSLAM running on base station*/
    CRobotRedisClient* mRedis;

    /** Info from Redis server **/
	double mFrameTime;
	int mTargetFound;
    double mTargetHeight;
	double mTargetZ;
	CPose2d mTargetPose;
	CPose2d mTargetPrevPose;
	CPose2d mPose;
	CPose2d mNNPose;

	double VEL;
	double MAX_VEL;
	double MIN_VEL;

	int readConfigFile(std::string& filename);
#ifdef STAGE
	/** Additional Stage objects */
    Stg::Model* stgTarget;
	Stg::Model* stgNN;
#endif
};

#endif
