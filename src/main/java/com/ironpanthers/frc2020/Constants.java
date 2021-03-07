/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {
        /* disallow construction of this class */
        throw new UnsupportedOperationException("don't try to construct an instance of Constants");
    }

    /**
     * When competition mode is enabled, most throwable errors are disregarded.
     * Classes in the project should use the competition mode variable in order to
     * decide what their failure behavior should be.
     */
    public static final boolean kCompetitionMode = false;

    public static final int kFalconEPR = 4096;
    public static final int kFalconCPR = 2048;
	public static class Auto {
		public static final int kAutoSelectorPort = 1;
	}

    public static class Drive {
        public static final int kLeft1Id = 1;
        public static final int kLeft2Id = 21;
        public static final int kRight1Id = 2;
        public static final int kRight2Id = 22;
		
		public static final int kShiftPort = 5;

        public static final int kShifterPCMId = 5;

        public static final double kGearRatio = 5.1;
        public static final double kTrackWidthMeters = 0.6579; // empirical from characterization data
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

        // GENERATE FROM CHARACTERIZATION TOOL
        // does not match with 2021 version robot (values are Bad)
        // UPDATE THIS COMMENT IF YOU CHANGE ANY OF THE DRIVEBASE GAINS
        public static final double kS = 0.319;
        public static final double kV = 1.27;
        public static final double kA = 0.348;

        public static final double kP = 1.37;

        public static final double kCurrentLimit = 20.0; // amps
        public static final double kCurrentTrigger = 40.0;
        public static final double kCurrentLimitSeconds = 0.5;
        public static final double kRampRate = 0.5; // seconds to full power
    }

    public static class OI {
		// Driver A
		public static final int kDriverAJoystickPort = 0;
        public static final int kDriveShiftButton = 2;
        
        public static final int kIntakeButton = 4;
        public static final int kTurnFineLeft = 7;
        public static final int kTurnFineRight = 8;

        // Driver B
        public static final int kDriverBJoystickPort = 1;
        public static final int kManualArmButton = 1;
        public static final int kDriverBIntakeButton = 2;
        
        public static final int kEmergencyOuttakeButton = 3;
        public static final int kStopShooterButtonB = 4;
        public static final int kShootFar = 5;
        public static final int kShootClose = 6;
        public static final int kEmergencyShootButton = 8;
        public static final int kControlPanel = 9;
        public static final int kShootInitiation = 10;
		public static final int kCloseTrenchButton = 11;
        public static final int kCloseShotButton = 12;
		
    }

    public static class Conveyor {
        // Ports
        public static final int kConveyorMotorId = 4;
        public static final int kIntakeMotorId = 3;
		public static final int kBannerSensorPort = 2;
		
		public static final boolean kIntakeInverted = true;

        // PID
        public static final int kPIDIdx = 0;
        public static final double kConveyorClosedLoopRamp = 0.6;
        public static final double kConveyorPositionKp = 0.216;

        // Powers TODO AAAAAAAAAA DOCUMENTATION
        public static final double kIntakeRollerSpeed = 1.0;
		public static final double kIntakeFlywheelSpeed = -0.6; // tbd
		public static final double kOuttakeRollerSpeed = -0.6;
        public static final double kConveyorSpeedClose = 0.6;
        public static final double kConveyorSpeedFar = 0.25;

		public static final double kConveyorTime = 1.9; // Seconds, needs testing

        // Encoder Stuff
        public static final int kPositionErrorTolerance = 350;
        public static final int kShiftEncoderDistance = 26000;
		public static final int kShiftEncoderDistanceLast = 11000;
		
		public static final int kSecondBallModifier = -1000;
		public static final int kThirdBallModifier = 10000;
		public static final int kFourthBallModifier = 15500;

    }

    public static class Vision {

        public static final String kLimelightName = "limelight-a"; 

        /**
         * Height from ground to pivot in inches
         */
        public static final double kGroundToPivotInches = 10;

        /**
         * Height from ground to target in inches
         */
        public static final double kGroundToTargetInches = 98.25; //Ceneter of target to ground
        
        /**
         * Angle from mount to limelight in degrees
         */
        public static final double kMountToLLAngleDeg = 30;

        public static final double kPivotToLL = 26.06; //Pivot point to limelight hypotonuse 
        public static double kPivotToLLAngle = 30.9267; //inverse cos of kPivotToLLPlane / kPivotToLL

        /**
         * Proportional control constant
         */
        public static final double kP = 0.008;

        /**
         * I value in PID
         */
        public static final double kI = 0.00;
        /**
         * D value in PID
         */
        public static final double kD = 0.000;
        /**
         * Minimum percent output required to break static friction
         */
        public static final double kS = 0.08;

        /** Conversion constant estimating magnitude of top line as it moves further away */
        public static final double kTopLineMagnitudeTimesDistance = 100 * 118.7; //TODO measure for practice field

        public static final double kTargetWidthInches = 30; //TODO measure for practice field

        /** Conversion constant relating distance from outer goal to hole to the length of the line on the top of the outer goal target*/
        //This was taken from field measurements. The outer hole is 2 ft 5.25 inches in front of the inner hole, and the diameter of the hexagon is 2 ft 6 inches
        public static final double kOuterToHoleDistancePerTlLength = 29.25 / 30;

        public static double kAutoAlignTolerance = .5;
        
        
		//public static final double X_ADJUST_PER_DEGREE = 0; //TODO measure (not used currently)
    }

    public static class Shooter {
        public static final int kShooter1Id = 5;
        public static final int kShooter2Id = 6;
		public static final int kShooter3Id = 7;
		
		public static final boolean IS_SHOOTER_INVERTED = true;

        public static final int kPIDIdx = 0;
        public static final double kF = 0.051;
        public static final double kP = 0.2;
        public static final double kRampRate = 0.25; // seconds 0->full

		public static final int kInnerGoalThreshold = 100; // Good for auto, too slow for tele
		public static final int kOuterGoalThreshold = 500; // When speed is more important than accuracy, 750 ok, 2000 ok close
        
        public static final int kCloseVelocity = 10000; // Tested 2/29/20, horizontal distance: 
        public static final double kCloseDistance = 40.0; // Tested 3/1/20

        public static final int kInitiationVelocity = 12000; // Tested 2/29/20
        public static final double kInitiationDistance = 135.0; // Tested 2/29/20

        public static final int kCloseTrenchVelocity = 13500; // Tested 3/10/20
        public static final double kCloseTrenchDistance = 170.0; // Needs Testing

        public static final int kFarVelocity = 15500; // Needs Testing
        public static final double kFarDistance = 310.0; // 2/29/20

		public static final double kCurrentLimit = 40; // amps
		
		public static final double kIntakeMotorSpeed = -.5;
    }

    public static class Arm {
        public static final int kLeftMotorId = 9;
        public static final int kRightMotorId = 10;
        public static final int kHighLimitSwitchPort = 1;
        public static final int kGroundLimitSwitchPort = 0;

        public static final int kCANCoderId = 0;
        public static final int kRemoteSensorSlot = 0; // RemoteSensor0 for CANCoder
        public static final double kCANCoderOffset = 0.0; // 0.5 works, but too risky to have a negative angle which can cause raw units to be almost 4096

		public static final int kBrakePort = 4; // TBD
		
		// CANCoder facing the opposite direction, so the two are inverted compared to each other
		public static final boolean kCANCoderPhase = true;
		public static final boolean kArmFalconPhase = false;

        public static final int kPIDIdx = 0;
        public static final double kArmAngleOffset = 0; // TODO find this value
        public static final double kArmInitialHeight = 0; // TODO find this value
        public static final double kMaxManualSpeed = 0.5;

        public static final double kP = 5.5; // Up is positive encoder direction, but negative voltage (3/5/20)
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double kRampRate = 0.25; // seconds 0->full

        // units
		public static final double kCanCoderCoefficient = 360.0 / 4096.0; // 4096 units per rotation, 360 degrees per rotation for CANCoder. Should be 2pi / 4096 for radians
        // Setpoints
        public static final double kPositionErrorTolerance = 0.5;

        public static final double kCloseShotDegrees = 13.0; //goes to 13
        public static final double kInitiationLineDegrees = 42.0; 
        public static final double kCloseTrenchDegrees = 46.0; //goes to 51
        public static final double kFarShotDegrees = 54.0; // Tested angle for shooting behind control panel at
                                                                   // 14k native
        // units
        public static final double kFrameHeightDegrees = 53.00; // Height at which robot is 45 inches tall

		// Soft Limits
		public static final double kClimbDegrees = 69.0; // 71.75 calculated arm angle would make the climb hooks go to a max of 11.99 inches outside frame perimeter
        public static final double kTopPositionDegrees = 78.0; // 90 degrees, should be close to top position
        public static final int kBottomSoftLimit = (int) (1.0 / kCanCoderCoefficient); // Convert into native units
		public static final int kTopSoftLimitEndgame = (int) ((kTopPositionDegrees - 1) / kCanCoderCoefficient); // Convert into native units
		public static final int kTopSoftLimit = (int) (kFrameHeightDegrees / kCanCoderCoefficient);
        public static final double kUseTopLimitRange = 40.0;

        // TODO(james)
        public static final double kClosedLoopPeakOutput = 0.5; // Used for both positive and negative direction

        public static final double kCurrentLimit = 60; // amps
	}
}
