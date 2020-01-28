/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

import static edu.wpi.first.wpilibj.util.Units.inchesToMeters;

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

    /*
     * UNIVERSAL CONSTANTS
     */
    public static final int kFalconTicksToRevs = 4096; // TODO(ingi)

    public static class Drive {
        public static final int kLeft1Id = 1;
        public static final int kLeft2Id = 21;
        public static final int kRight1Id = 2;
        public static final int kRight2Id = 22;

        public static final int kPigeonTalonId = 31;

        public static final double kGearRatio = 5.10; // TODO(ingi)
        public static final double kTrackWidthMeters = inchesToMeters(21); // TODO(ingi)
        public static final double kWheelRadiusMeters = inchesToMeters(3);

        // GENERATE FROM CHARACTERIZATION TOOL TODO(ingi)
        // LAST GENERATED: NEVER
        // BY: NO ONE
        // UPDATE THIS COMMENT IF YOU CHANGE ANY OF THE DRIVEBASE GAINS
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kP = 0;
    }

    public final class OI {
        public static final int JOYSTICK_PORT = 0;
        public static final int INTAKE_BUTTON_PORT = 4;
		public static final int RESET_CONVEYOR_BUTTON_PORT = 3;
		public static final int SHOOT_WITH_VELOCITY_PORT = 5;
    }

    public final class Conveyor {
        // Ports
        public static final int CONVEYOR_BELT_MOTOR_PORT = 4;
        public static final int INTAKE_MOTOR_PORT = 3;
        public static final int BANNER_SENSOR_PORT = 0;

        // Size Constants
        public static final double POWER_CELL_DIAMETER = 7; // in inches

        // Powers
        public static final double CONVEYOR_BELT_MOTOR_POWER = 0.5; // tbd
        public static final double INTAKE_MOTOR_POWER = -1; // tbd
        public static final double SHOOTER_MOTOR_POWER = -.5; // tbd

        // Encoder Stuff
        public static final double DISTANCE_PERENCODER_ROTATION = .1;
        public static final double PULSES_PERENCODER_ROTATION = .25;
        public static final int TICK_ERROR_TOLLERANCE = 700;
        /** needs to move conveyer exactly one ball length backward */
        public static final int TICKS_PREP_DISTANCE = -21000;
    }

    public final class Vision {
        /** Height from ground to limelight in inches */
        public static final double HEIGHT_GROUND_TO_LIMELIGHT = 38.5;
        /** Height from ground to target in inches */
        public static final double HEIGHT_GROUND_TO_TARGET = 93.0;
        /** Angle from mount to limelight in degrees */
        public static final double ANGLE_MOUNT_TO_LIMELIGHT = 33.0;
        /** Proportional control constant */
        public static final double Kp = -0.1;
        /** I value in PID */
        public static final double Ki = 0.01;
        /** D value in PID */
        public static final double Kd = 0.001;
        /** Minimum amount of power to move robot */
        public static final double MINIMUM_POWER = 0.03;
        /** Button port to adjust angle of robot */
        public static final int ANGLE_ADJUSTING_BUTTON_PORT = 9;
    }

    public final class Shooter {
        public static final int SHOOTER_ONE_PORT = 5;
        public static final int SHOOTER_TWO_PORT = 6;
        public static final int SHOOTER_THREE_PORT = 7;
        public static final double shooterSpeed = 0.5;
        public static final int SHOOTER_VELOCITY_IDX = 0;
        public static final double SHOOTER_CURRENT_LIMIT = 40;
        public static final double SHOOTER_F = 0.053;
        public static final double SHOOTER_P = 0.2;
        public static final double SHOOTER_MAX_SAFE_VEL = 12000; // Native units
		public static final double SHOOTER_RAMP_RATE = 0.25; // Seconds to full power during PID control + Open Loop
		public static final int SHOOTER_VELOCITY_THRESHOLD = 300; // Acceptable error in velocity before shooting
		public static final int SHOOTER_TEST_VELOCITY = 15000;
	}
	
	public final class Arm {
		public static final int ARM_LEFT_PORT = 9;
		public static final int ARM_RIGHT_PORT = 10;
		public static final boolean IS_LEFT_ARM_INVERTED = true;
		public static final boolean IS_RIGHT_ARM_INVERTED = false;
		public static final int ARM_POSITION_PID_SLOT = 0;
        public static final int ARM_VELOCITY_PID_SLOT = 1;
        public static final double TICKS_TO_DEGREES = 360 * 4096;
        public static final double ARM_ANGLE_OFFSET = 0; //TODO find this value
        public static final double ARM_INITIAL_HEIGHT = 0; //TODO find this value
        public static final double MAX_FF = 0.07;

		public static final double ARM_POSITION_P = 0;
		public static final double ARM_POSITION_I = 0;
		public static final double ARM_POSITION_D = 0;
        public static final double ARM_POSITION_F = 0;
        
        public static final int REVERSE_LIMIT_SWITCH_PORT = 0;
        public static final int FORWARD_LIMIT_SWTICH_PORT = 0;
        
        public static final double LIMIT_SWITCH_P = 0.1;

	}
}
