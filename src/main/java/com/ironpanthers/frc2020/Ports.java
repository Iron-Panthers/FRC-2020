package com.ironpanthers.frc2020;

/**
 * Ports contains all IDs on the robot. While similar to constants in theory, it
 * should ONLY contain the hardware/firmware level IDs for electronic components
 * of the robot (CAN bus ids, PCM ids, etc.).
 */
public final class Ports {

    private Ports() {
        /* disallow construction of this class */
        throw new UnsupportedOperationException("don't try to construct an instance of Ports");
    }

    // public static final int kMyPort = 1;

    public static final int kCANDriveLeft1 = 2;
    public static final int kCANDriveLeft2 = 22;
    public static final int kCANDriveRight1 = 1;
    public static final int kCANDriveRight2 = 21;

    public static final int kCANPigeonTalon = 5;
}
