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
    public final class Vision {
        /** Height from ground to limelight in inches */
        public static final double HEIGHT_GROUND_TO_LIMELIGHT = 38.5;
        /** Height from ground to target in inches */
        public static final double HEIGHT_GROUND_TO_TARGET = 93.0;
        /** Angle from mount to limelight in degrees */
        public static final double ANGLE_MOUNT_TO_LIMELIGHT = 33.0;
        /** Proportional control constant */
        public static final double Kp = 0.1;
        /** I value in PID */
        public static final double Ki = 0.01;
        /** D value in PID */
        public static final double Kd = 0.001;
        /** Minimum amount of power to move robot */
        public static final double MINIMUM_POWER = 0.03;
        /** Button port to adjust angle of robot */
        public static final int ANGLE_ADJUSTING_BUTTON_PORT = 9;
        /** Conversion constant estimating magnitude of top line as it moves further away */
        public static final double TOP_LINE_MAGNITUDE_AT_DISTANCE = 0; //TODO measure
        /** Conversion constant estimating magnitude of bottom line as it moves further away */
        public static final double BOTTOM_LINE_MAGNITUDE_AT_DISTANCE = 0; //TODO measure
        /** Conversion constant relating distance from outer goal to hole to the length of the line on the top of the outer goal*/
        public static final double OUTER_TO_HOLE_DISTANCE_PER_TL_LENGTH = 0; //TODO measure
        /**Conversion constant relating distance from outer goal to hole to the length of the line between the bottom two outer goal points*/
        public static final double OUTER_TO_HOLE_DISTANCE_PER_BL_LENGTH = 0; //TODO measure
        /** Converts between degrees off from center of outer goal to inner hole to a usable horizontal error for turning */
        public static final double X_ADJUST_PER_DEGREE = 0; //TODO measure

    }
    public final class Shooter {
        public static final int SHOOTER_ONE_PORT = 0;
        public static final int SHOOTER_TWO_PORT = 0;
		public static final int SHOOTER_THREE_PORT = 0;

		public static final int SHOOTER_VELOCITY_IDX = 0;
    }
   

    /**
     * When competition mode is enabled, most throwable errors are disregarded.
     * Classes in the project should use the competition mode variable in order to
     * decide what their failure behavior should be.
     */
    public static final boolean kCompetitionMode = false;
}
