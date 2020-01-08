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

    public static final double h1 = 5.0; // Height from ground to limelight (unofficial)
    public static final double h2 = 10.0; // Height from ground to target (unofficial)
    public static final double a1 = 25.0; // Angle from mount to limelight in degrees (unofficial)
    public static final double Kp = -0.1; // Proportional control constant (not tuned)
    public static final double Ki = 0.01; //I value in PID (not tuned)
    public static final double Kd = 0.001; //D value in PID (not tuned)
    public static final double MINIMUM_POWER = 0.03; // Minimum amount of power to move robot (not tuned)
    public static final int ANGLE_ADJUSTING_BUTTON_PORT = 9; // Button port to adjust angle of robot

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
}
