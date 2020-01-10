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

    /*
     * DRIVEBASE CONSTANTS
     */

    public static final double kGearRatio = 5.10; // TODO(ingi)
    public static final double kTrackWidthMeters = inchesToMeters(21); // TODO(ingi)
    public static final double kWheelRadiusMeters = inchesToMeters(3);

    // GENERATE FROM CHARACTERIZATION TOOL TODO(ingi)
    // LAST GENERATED: NEVER
    // BY: NO ONE
    // UPDATE THIS COMMENT IF YOU CHANGE ANY OF THE DRIVEBASE GAINS
    public static final double kDriveKs = 0;
    public static final double kDriveKv = 0;
    public static final double kDriveKa = 0;

    public static final double kDriveKp = 0;
}
