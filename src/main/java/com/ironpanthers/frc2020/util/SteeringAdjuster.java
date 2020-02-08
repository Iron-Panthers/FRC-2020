/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.util;

import com.ironpanthers.frc2020.Constants;

/**
 * Add your docs here.
 */
public class SteeringAdjuster {

    private double[] totalErrors;
    private double sumOfErrors;
    private double lastError;
    private double deltaError;

    private double adjustedSteeringValue;
    private double leftSteeringAngle;// power adjustment for left steering
    private double rightSteeringAngle; // power adjustment for right steering

    public SteeringAdjuster() {
        totalErrors = new double[100];
        sumOfErrors = 0.0;
        lastError = 0.0;
        deltaError = 0.0;

        adjustedSteeringValue = 0.0;
        leftSteeringAngle = 0.0;
        rightSteeringAngle = 0.0;
    }

    private boolean targetVisible() {
        return LimelightWrapper.getLimelightWrapperFront().getTableV() == 1 && 

    }

    public void updateSteeringValues(double x) {
        //positive error should indicate the target is on the right of the screen 
        double horizontalError = x;

        //shift recorded error values over and set zeroth slot to the measured error
        for (int i = 98; i >= 0; i--) {
            totalErrors[i + 1] = totalErrors[i];
        }
        totalErrors[0] = horizontalError;

        //sum errors
        for (int i = 0; i < totalErrors.length; i++) {
            sumOfErrors += totalErrors[i];
        }

        //calculate error delta since last recording
        deltaError = horizontalError - lastError;
 
        if (!targetVisible()) {
            adjustedSteeringValue = 0.0; 
        } 
        else {
            if (x > 0.0) {
                adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                        + Constants.Vision.kD * deltaError - Constants.Vision.kS;
            } else if (x < 0.0) {
                adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                        + Constants.Vision.kD * deltaError + Constants.Vision.kS;
            }
        }

        //reset last recorded error to error just recorded
        lastError = horizontalError;
    }

    public double getLeftSteeringAdjust() {
        return adjustedSteeringValue;
    }

    public double getRightSteeringAdjust() {
        return -adjustedSteeringValue;
    }
}
