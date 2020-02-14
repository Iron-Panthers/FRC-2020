package com.ironpanthers.util;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utilities for Phoenix motor controllers.
 * 
 * @author Ingi Helgason
 */
public class PhoenixUtil {
    
    private PhoenixUtil() {
        /* disallow init of this class */
        throw new UnsupportedOperationException("don't try to construct an instance of PhoenixUtil");
    }

    /**
     * Reads an ErrorCode value, and reports an error if the ErrorCode indicates a
     * notable issue.
     * 
     * @param error   The error code to read (result of Phoenix functions).
     * @param message The message to print if error is not "OK".
     */
    public static void checkError(ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError(message + error, false);
        }
    }
}