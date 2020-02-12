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

    public static class Drive {
        public static final int kLeft1Id = 1;
        public static final int kLeft2Id = 21;
        public static final int kRight1Id = 2;
        public static final int kRight2Id = 22;

        public static final double kGearRatio = 5.10;
        public static final double kTrackWidthMeters = 0.7204688778663988; // empirical from characterization data
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

        // GENERATE FROM CHARACTERIZATION TOOL
        // LAST GENERATED: 2020-02-04
        // BY: Ingi Helgason (helgason.ingi@gmail.com)
        // UPDATE THIS COMMENT IF YOU CHANGE ANY OF THE DRIVEBASE GAINS
        public static final double kS = 0.252;
        public static final double kV = 2.22;
        public static final double kA = 0.207;

        public static final double kP = 2.6;

        public static final double kCurrentLimit = 60; // amps
    }

    public static class OI {
        public static final int kCloseShotButtonNumber = 12;
        public static final int kInitiationLineShotButtonNumber = 10;
        public static final int kFarShotButtonNumber = 8;
        // Driver A
        public static final int kStopShooterButton = 3;
        public static final int kDriverAJoystickPort = 0;
        public static final int kIntakeButton = 4;
        public static final int kResetConveyorButton = 3;
        public static final int kShootFar = kFarShotButtonNumber;
        public static final int kAutoAlign = 6;
        public static final int kShootClose = kCloseShotButtonNumber;
        public static final int kShootInitiation = kInitiationLineShotButtonNumber;

        // Driver B
        public static final int kDriverBJoystickPort = 1;
        public static final int kManualArmButton = 1;
		public static final int kDriverBIntakeButton = 2;
		public static final int kEmergencyOuttakeButton = 3;
        public static final int kZeroArmButton = 7;
        public static final int kCloseShotButton = kCloseShotButtonNumber;
        public static final int kFarShotButton = kFarShotButtonNumber;
        public static final int kFramePerimeterHeightButton = 9;
        public static final int kAutoShotHeightButton = kInitiationLineShotButtonNumber;
        public static final int kEmergencyintakeButton = 5;
    }

    public static class Conveyor {
        // Ports
        public static final int kConveyorMotorId = 4;
        public static final int kIntakeMotorId = 3;
        public static final int kBannerSensorPort = 3;

        // PID
        public static final int kPIDIdx = 0;
        public static final double kConveyorClosedLoopRamp = 0.6;
        public static final double kConveyorKp = 0.13;

        // Powers
        public static final double kIntakeRollerSpeed = 1.0;
		public static final double kIntakeFlywheelSpeed = -.5; // tbd
		public static final double kOuttakeRollerSpeed = -1.0;

        // Encoder Stuff
        public static final int kPositionErrorTolerance = 350;
        public static final int kShiftEncoderDistance = 16000;
    }

    public static class Vision {
        /**
         * Height from ground to pivot in inches
         */
        public static final double kGroundToPivotInches = 10.3;

        public static final double kLimelightToPivotPlaneInches = 8;
        /**
         * Height from ground to target in inches
         */
        public static final double kGroundToTargetInches = 98.0;
        
        /**
         * Angle from mount to limelight in degrees
         */
        public static final double kMountToLLAngleDeg = 20.0;

        public static final double kPivotToLLDeg = 23.5;

        /**
         * Proportional control constant
         */
        public static final double kP = 0.1;
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
        public static final double kS = 0.03;

        /** Conversion constant estimating magnitude of top line as it moves further away */
        public static final double kTopLineMagnitudeTimesDistance = 51 * 201.825; //TODO measure for practice field

        /** Conversion constant relating distance from outer goal to hole to the length of the line on the top of the outer goal target*/
        //This was taken from field measurements. The outer hole is 2 ft 5.25 inches in front of the inner hole, and the diameter of the hexagon is 2 ft 6 inches
        public static final double kOuterToHoleDistancePerTlLength = 29.25 / 30; 
        
        //public static final double X_ADJUST_PER_DEGREE = 0; //TODO measure (not used currently)
    }

    public static class Shooter {
        public static final int kShooter1Id = 5;
        public static final int kShooter2Id = 6;
        public static final int kShooter3Id = 7;

        public static final int kPIDIdx = 0;
        public static final double kF = 0.051;
        public static final double kP = 0.2;
        public static final double kRampRate = 0.25; // seconds 0->full

        public static final int kVelocityThreshold = 100; // Good for auto, too slow for tele
        public static final int kTestVelocity = 18000;
        public static final int kCloseVelocity = 15000; // Tested 2/11/20 by James
        public static final int kFarVelocity = 19000;
        public static final int kInitiationVelocity = 16500; // Initiation Line, Tested 2/11/20

        public static final double kCurrentLimit = 40; // amps
    }

    public static class Arm {
        public static final int kLeftMotorId = 9;
        public static final int kRightMotorId = 10;
        public static final int kHighLimitSwitchPort = 1;
        public static final int kGroundLimitSwitchPort = 0;

        public static final int kPIDIdx = 0;
        public static final double kArmAngleOffset = 0; // TODO find this value
        public static final double kArmInitialHeight = 0; // TODO find this value
        public static final double kHorizontalHoldOutput = 0.08; // 0.125 tested 2/11/20
        public static final double kMaxManualSpeed = 0.5;

        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double kRampRate = 0.25; // seconds 0->full

        // Setpoints
        public static final int kPositionErrorTolerance = 250;
        public static final int kInitiationLineHeight = 47000; // Tested 2/11/20 (45400)
        public static final int kCloseShotHeightNativeUnits = 16000; // 19 too high at 15k velocity, 15 almost too low
        public static final int kFarShotHeightNativeUnits = 60000; // Tested angle for shooting behind control panel at
                                                                   // 14k native
        // units
        public static final int kFrameConstrainedHeightNativeUnits = 45000; // Height at which robot is 45 inches tall
        public static final double encoderToAngle = 90.0 / 79300; // Empirically tested conversion
        public static final double kDegreesPerOutputRotation = 360.0;
        public static final double kArmReduction = 175.0; // Falcon rev to arm rev
        // public static final double encoderToAngle = kDegreesPerOutputRotation / (kFalconCPR * kArmReduction); // Theoretical

        // Soft Limits
        public static final int kTopPositionNativeUnits = 88000; // Tested by James, 1/30/20
        public static final int kBottomSoftLimit = 0;
        public static final int kTopSoftLimit = kTopPositionNativeUnits + 500;

        // TODO(james)
        public static final int kSlowdownThreshold = 10000; // Threshold to soft limit in which the output of the arm
        // motors are scaled down
        public static final int kBottomSlowdownZone = kBottomSoftLimit + kSlowdownThreshold;
        public static final int kTopSlowdownZone = kTopSoftLimit - kSlowdownThreshold;
        public static final double kSlowClosedLoopPeakOutput = 0.25; // for when in a slow zone
        public static final double kClosedLoopPeakOutput = 0.5; // Used for both positive and negative direction

        public static final double kCurrentLimit = 60; // amps
    }
}
