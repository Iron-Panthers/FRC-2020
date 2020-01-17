package com.ironpanthers.frc2020;

/**
 * Ports contains all IDs on the robot. While similar to constants in theory, it
 * should ONLY contain the hardware/firmware level IDs for electronic components
 * of the robot (CAN bus ids, PCM ids, etc.).
 */
public final class Ports {

    public static final int DIGITAL_INPUT_PORT = 0;
    public static final int CONVEYOR_BELT_MOTOR_PORT = 0;
    private Ports() {
        /* disallow construction of this class */
        throw new UnsupportedOperationException("don't try to construct an instance of Ports");
    }

    // public static final int kMyPort = 1;
}
