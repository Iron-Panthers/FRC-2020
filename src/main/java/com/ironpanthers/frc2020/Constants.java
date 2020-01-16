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

    public static final double SHOOTER_SPEED = 0.5;
	public static final boolean kCompetitionMode = false;
	public static final int SHOOTER_VELOCITY_IDX = 0;
    public static final double SHOOTER_CURRENT_LIMIT = 40;
    
    //PID testing constants
    public static final double RADIANS_TO_DEGREES = Math.PI/180;
    public static final double METER_TO_FOOT = 3.28084;
    public static final double PID_TESTING_DIAGONAL_TO_SPEED = 0.03333333333; 
    public static final double ANGLE_MOUNT_TO_LIMELIGHT = 35.0;
    public static final double GRAVITY_CONSTANT = 9.80665;
    public static final double SHOOTER_ANGLE = 35; 
    //TODO get conversion from power percentage to m/s
}
