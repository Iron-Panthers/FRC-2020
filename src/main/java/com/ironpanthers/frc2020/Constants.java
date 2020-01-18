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
    //TODO figure out how to get shooter speed based on distance
    public final class OI {
        public static final int JOYSTICK_PORT = 0;
        public static final int INTAKE_BUTTON_PORT = 0;
    }
    public final class Conveyor {
        public static final double CONVEYOR_BELT_MOTOR_POWER = 0.5; //tbd
        public static final double POWER_CELL_DIAMETER = 7; //in inches
		public static final int DIGITAL_INPUT_PORT = 0; //tbd
		public static final int CONVEYOR_BELT_MOTOR_PORT = 0; //tbd
		public static final double DISTANCE_PERENCODER_ROTATION = .1;
		public static final double PULSES_PERENCODER_ROTATION = .25;
		public static final double INTAKE_MOTOR_SPEED = 0; //tbd
        public static final double SHOOTER_MOTOR_SPEED = 0; //tbd
		public static final double PREPARATION_DISTANCE = 0; //tbd
		public static final double DEFAULT_DISTANCE = 0; //tbd
		public static final int CANCODER_PORT = 0;
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
        public static final int SHOOTER_ONE_PORT = 1;
        public static final int SHOOTER_TWO_PORT = 2;
		public static final int SHOOTER_THREE_PORT = 3;
        public static final double shooterSpeed = 0.5;
        public static final boolean kCompetitionMode = false;
        public static final int SHOOTER_VELOCITY_IDX = 0;
        public static final double SHOOTER_CURRENT_LIMIT = 40;
        public static final double SHOOTER_F = 0.053;
        public static final double SHOOTER_P = 0.2;
        public static final double SHOOTER_MAX_SAFE_VEL = 12000; //Native units
        public static final double SHOOTER_RAMP_RATE = 0.25; // Seconds to full power during PID control + Open Loop
    }

}
