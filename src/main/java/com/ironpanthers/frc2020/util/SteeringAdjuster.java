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
import com.ironpanthers.frc2020.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SteeringAdjuster {

    private double[] totalErrors;
    private double sumOfErrors;
    private double lastError;
    private Arm arm;
    private double deltaError;

    public double adjustedSteeringValue;
    public double adjustValue;
    LimelightWrapper limelight;

    private double lastComputedAngle = 0.0;

    DoubleSupplier horizontalDistance, diagonalDistance;

    // readd DoubleSupplier horizontalDistance, DoubleSupplier diagonalDistance,
    // eventually
    public SteeringAdjuster(LimelightWrapper limelight, DoubleSupplier horizontalDistance, Arm arm) {
        totalErrors = new double[100];
        sumOfErrors = 0.0;
        lastError = 0.0;
        deltaError = 0.0;

        adjustedSteeringValue = 0.0;
        this.limelight = limelight;
        this.arm = arm;
        this.horizontalDistance = horizontalDistance;
    }

    /**
     * updates the steering adjustment value which can then be added to left motor
     * power and subtracted from right.
     * 
     * @param aimDeg the degrees off of the center of the screen which the limelight
     *               should aim at.
     */
    public void updateSteeringValues() {
        limelight.periodic();

        // positive error should indicate that the part of the image we are aiming at is
        // to the right of the crosshair
        double horizontalError = limelight.getTableX();
        SmartDashboard.putNumber("Alignment Adjust Angle", horizontalError);

        // shift recorded error values over and set zeroth slot to the measured error
        for (int i = 98; i >= 0; i--) {
            totalErrors[i + 1] = totalErrors[i];
        }
        totalErrors[0] = horizontalError;

        // sum errors
        for (int i = 0; i < totalErrors.length; i++) {
            sumOfErrors += totalErrors[i];
        }

        // calculate error delta since last recording
        deltaError = horizontalError - lastError;

        // calculate steering adjustment with pidf
        if (horizontalError > 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError + Constants.Vision.kS;
        } else if (horizontalError < 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError - Constants.Vision.kS;
        }
        // reset last recorded error to error just recorded
        lastError = horizontalError;
    }
    public double updateSteeringValuesI() {
        limelight.periodic();
         return getInnerHoleAdjust() * Constants.Vision.kP;
    }
    public void updateSteeringValuesW() {
        limelight.periodic();

        // positive error should indicate that the part of the image we are aiming at is
        // to the right of the crosshair
        double horizontalError = getInnerHoleAdjust();
        SmartDashboard.putNumber("Alignment Adjust Angle", horizontalError);

        // shift recorded error values over and set zeroth slot to the measured error
        for (int i = 98; i >= 0; i--) {
            totalErrors[i + 1] = totalErrors[i];
        }
        totalErrors[0] = horizontalError;

        // sum errors
        for (int i = 0; i < totalErrors.length; i++) {
            sumOfErrors += totalErrors[i];
        }

        // calculate error delta since last recording
        deltaError = horizontalError - lastError;

        // calculate steering adjustment with pidf
        if (horizontalError > 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError + Constants.Vision.kS;
        } else if (horizontalError < 0.0) {
            adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                    + Constants.Vision.kD * deltaError - Constants.Vision.kS;
        }
        // reset last recorded error to error just recorded
        lastError = horizontalError;
    }

    // If the target is on the right of the screen, the steering value will be
    // positive.
    // Therefore, we add to left side and subtract from right
    public double getLeftSteeringAdjust() {
        return adjustedSteeringValue;
    }

    public double getRightSteeringAdjust() {
        return -adjustedSteeringValue;
    }
    public double getLeftSteeringAdjustI() {
        return updateSteeringValuesI();
    }
    public double getRightSteeringAdjustI() {
        return -updateSteeringValuesI();
    }

    /**
     * @return the angle off the center of the vision target which we need to aim in
     *         order to hit the inner hole. Positive if the inner hole will appear
     *         to the right of the target's center.
     */
    public double getInnerHoleAdjust() {
        double[] tcornxy = limelight.getTCornXY();

        // Kill self if cannot find 4 x,y pairs (math breaks down when not 4 corners are present)
        if (tcornxy.length != 8) {
            System.out.println("Could not find 8 corners! \nInstead, there were only " + tcornxy.length + " corners.\nReturning last computed value...\n\n\n");
            return lastComputedAngle;
        }

        double[] tcornx = { tcornxy[0], tcornxy[2], tcornxy[4], tcornxy[6] }; // this breaks everything when arm isn't
                                                                              // fully stable
        double[] tcorny = { tcornxy[1], tcornxy[3], tcornxy[5], tcornxy[7] };

        double tleftx = 0; // x coord of top left corner of hexagon
        double tlefty = 0;
        double trightx = 0;
        double trighty = 0;
        double bleftx = 0;
        double blefty = 0;
        double brightx = 0;
        double brighty = 0;

        ArrayList<Double> orderedx = new ArrayList<Double>();
        ArrayList<Double> orderedy = new ArrayList<Double>();

        // keeps associated x and y values from tcornx and tcorny at the same index as
        // each other in the new lists.
        // orders the list from lowest to highest x
        orderedx.add(tcornx[0]);
        orderedy.add(tcorny[0]);
        for (int i = 1; i < tcornx.length; i++) {
            int size = orderedx.size();
            for (int n = 0; n < size; n++) {
                if (tcornx[i] < orderedx.get(n)) {
                    orderedx.add(n, tcornx[i]);
                    orderedy.add(n, tcorny[i]);
                    break;
                }
                if (n == size - 1) {
                    orderedx.add(tcornx[i]);
                    orderedy.add(tcorny[i]);
                }
            }

        }

        // now the corners have been identified as top left, bottom left, etc.
        tleftx = orderedx.get(0);
        tlefty = orderedy.get(0);
        bleftx = orderedx.get(1);
        blefty = orderedy.get(1);
        trightx = orderedx.get(3);
        trighty = orderedy.get(3);
        brightx = orderedx.get(2);
        brighty = orderedy.get(2);

        // // for testing:
        // SmartDashboard.putNumber("top left x: ", tleftx);
        // SmartDashboard.putNumber("top left y: ", tlefty);
        // SmartDashboard.putNumber("top right x: ", trightx);
        // SmartDashboard.putNumber("top right y: ", trighty);
        // SmartDashboard.putNumber("bottom left x: ", bleftx);
        // SmartDashboard.putNumber("bottom left y: ", blefty);
        // SmartDashboard.putNumber("bottom right x: ", brightx);
        // SmartDashboard.putNumber("bottom right y: ", brighty);
        SmartDashboard.putNumber("horizontal distance from target: ", horizontalDistance.getAsDouble());

        ThreeDimensionalSegment topLine = new ThreeDimensionalSegment(tleftx, tlefty, trightx, trighty,
                Constants.Vision.kTopLineMagnitudeTimesDistance / arm.getDiagonalDistance());
        // ThreeDimensionalSegment botLine = new ThreeDimensionalSegment(bleftx, blefty,
        // brightx, brighty, Constants.Vision.BOTTOM_LINE_MAGNITUDE_AT_DISTANCE /
        // calculateDiagonalDistance());

        // average of the estimated angles using the top and bottom lines
        double offsetAngle = topLine.getOffsetAngle(
                topLine.magnitude * Constants.Vision.kOuterToHoleDistancePerTlLength, arm.getHorizontalDistance2());

        // For debugging purposes, put the computed offset angle (SANS SGN! not reflective of the true target)
        SmartDashboard.putNumber("hole offset angle: ", offsetAngle);

        // Compute vertical delta between bottom, top corner on each side of the detection
        double leftHeight = Math.abs(tlefty - blefty);
        double rightHeight = Math.abs(trighty - brighty);

        // Update internal value
        lastComputedAngle = Math.copySign(offsetAngle, rightHeight - leftHeight);

        // Return stored value
        return lastComputedAngle;
    }
}
