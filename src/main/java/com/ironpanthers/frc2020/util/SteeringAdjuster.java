/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.util;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    LimelightWrapper limelight;

    DoubleSupplier horizontalDistance, diagonalDistance;
    // readd DoubleSupplier horizontalDistance, DoubleSupplier diagonalDistance, eventually
    public SteeringAdjuster(LimelightWrapper limelight) {
        totalErrors = new double[100];
        sumOfErrors = 0.0;
        lastError = 0.0;
        deltaError = 0.0;

        adjustedSteeringValue = 0.0;
        leftSteeringAngle = 0.0;
        rightSteeringAngle = 0.0;
        this.limelight = limelight;
        this.horizontalDistance = horizontalDistance;
        this.diagonalDistance = diagonalDistance;
    }

    public void updateSteeringValues() {
        limelight.periodic();

        //positive error should indicate the target is on the right of the screen 
        double horizontalError = limelight.getTableX();

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
 
        //calculate steering adjustment with pidf
        if (horizontalError > 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError + Constants.Vision.kS;
        } else if (horizontalError < 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError - Constants.Vision.kS;
        }

        //reset last recorded error to error just recorded
        lastError = horizontalError;
    }

    //If the target is on the right of the screen, the steering value will be positive.
    //Therefore, we add to left side and subtract from right
    public double getLeftSteeringAdjust() {
        return adjustedSteeringValue;
    }

    public double getRightSteeringAdjust() {
        return -adjustedSteeringValue;
    }

    // public double getInnerHolePixelAdjust() {

    //     double[] tcornxy = limelight.getTCornXY();
    //     double[] tcornx = {tcornxy[0], tcornxy[2], tcornxy[4], tcornxy[6]};
    //     double[] tcorny = {tcornxy[1], tcornxy[3], tcornxy[5], tcornxy[7]};

    //     double tleftx = 0; //x coord of top left corner of hexagon
    //     double tlefty = 0;
    //     double trightx = 0;
    //     double trighty = 0;
    //     double bleftx = 0;
    //     double blefty = 0;
    //     double brightx = 0;
    //     double brighty = 0;

    //     ArrayList<Double> orderedx = new ArrayList<Double>();
    //     ArrayList<Double> orderedy = new ArrayList<Double>();

    //     //keeps associated x and y values from tcornx and tcorny at the same index as each other in the new lists.
    //     //orders the list from lowest to highest x
    //     orderedx.add(tcornx[0]);
    //     orderedy.add(tcorny[0]);
    //     for (int i = 1; i < tcornx.length; i++) {
    //         int size = orderedx.size();
    //         for(int n = 0; n < size; n++) {
    //             if(tcornx[i] < orderedx.get(n)) {
    //                 orderedx.add(n, tcornx[i]);
    //                 orderedy.add(n, tcorny[i]);
    //                 break;
    //             }
    //             if(n==size-1) {
    //                 orderedx.add(tcornx[i]);
    //                 orderedy.add(tcorny[i]);
    //             }
    //         }
            
    //     }

    //     //now the corners have been identified as top left, bottom left, etc.
    //     tleftx = orderedx.get(0);
    //     tlefty = orderedy.get(0);
    //     bleftx = orderedx.get(1);
    //     blefty = orderedy.get(1);
    //     trightx = orderedx.get(3);
    //     trighty = orderedy.get(3);
    //     brightx = orderedx.get(2);
    //     brighty = orderedy.get(2);

    //     //for testing:
    //     SmartDashboard.putNumber("top left x: ", tleftx);
    //     SmartDashboard.putNumber("top left y: ", tlefty);
    //     SmartDashboard.putNumber("top right x: ", trightx);
    //     SmartDashboard.putNumber("top right y: ", trighty);
    //     SmartDashboard.putNumber("bottom left x: ", bleftx);
    //     SmartDashboard.putNumber("bottom left y: ", blefty);
    //     SmartDashboard.putNumber("bottom right x: ", brightx);
    //     SmartDashboard.putNumber("bottom right y: ", brighty);
    //     SmartDashboard.putNumber("diagonal distance from target: ", diagonalDistance.getAsDouble());
    //     SmartDashboard.putNumber("horizontal distance from target: ", horizontalDistance.getAsDouble());

    //     ThreeDimensionalSegment topLine = new ThreeDimensionalSegment(tleftx, tlefty, trightx, trighty, Constants.Vision.kTopLineMagnitudeTimesDistance / diagonalDistance.getAsDouble());
    //     //ThreeDimensionalSegment botLine = new ThreeDimensionalSegment(bleftx, blefty, brightx, brighty, Constants.Vision.BOTTOM_LINE_MAGNITUDE_AT_DISTANCE / calculateDiagonalDistance());

    //     //average of the estimated angles using the top and bottom lines
    //     double offsetAngle = topLine.getOffsetAngle(topLine.magnitude * Constants.Vision.kOuterToHoleDistancePerTlLength, horizontalDistance.getAsDouble());
        
    //     SmartDashboard.putNumber("hole offset angle: ", offsetAngle);

    //     double leftHeight = tlefty - blefty;
    //     double rightHeight = trighty - brighty;

    //     //copysign() to determine whether the adjust is going to be left or right
    //     return Math.copySign(offsetAngle, rightHeight - leftHeight);

    // }
}
