package com.ironpanthers.util;

public class Deadband {

    private Deadband() {
        /* disallow init of this class */
        throw new UnsupportedOperationException("don't try to construct an instance of Deadband");
    }

    private static double apply(double value, double deadband, boolean scaleDeadband, double maxMagnitude) {
        if (value >= -deadband && value <= deadband) {
            return 0;
        }
        if (!scaleDeadband) {
            return value;
        }
        return ((Math.abs(value) - deadband) / (maxMagnitude - deadband)) * Math.signum(value);
    }

    /**
     * Provides a scaled deadband for an HID axis value.
     * 
     * @param value    The HID axis value (1.0..1.0).
     * @param deadband The effective zero magnitude for the deadband.
     * @return The filtered value.
     */
    public static double apply(double value, double deadband) {
        return apply(value, deadband, true, 1.0);
    }
}