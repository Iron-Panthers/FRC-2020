/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
    private final Drive drive;

    private double x, v;
    private double[] totalErrors;
    private double lastError;
    private double sumOfErrors;
    private LimelightWrapper limelightWrapper;
    private double leftSteeringAngle;// Amount of degrees to steer left
    private double rightSteeringAngle; // Amount of degrees to steer right
    private double horizontalError; // Horizontal error from limelight to target
    private double adjustedSteeringValue; // Calculated amount of degrees to steer
    private double recentError;

    public TurnToTarget(Drive drive) {
        this.drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        totalErrors = new double[100];
        lastError = 0;
        sumOfErrors = 0;
        leftSteeringAngle = 0.0;
        rightSteeringAngle = 0.0;
        horizontalError = -x;
        adjustedSteeringValue = 0;
        recentError = 0;
        limelightWrapper = new LimelightWrapper();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        limelightWrapper.periodic();
        x = limelightWrapper.getX();
        v = limelightWrapper.getV();

        for (int i = 98; i >= 0; i--) {
            totalErrors[i + 1] = totalErrors[i];
        }
        totalErrors[0] = horizontalError;

        for (int i = 0; i < totalErrors.length; i++) {
            sumOfErrors += totalErrors[i];
        }

        recentError = horizontalError - lastError;

        if (v == 0.0) {
            adjustedSteeringValue = 0.3;
        } else {
            if (x > 0.0) {
                adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                        + Constants.Vision.kD * recentError - Constants.Vision.kS;
            } else if (x < 0.0) {
                adjustedSteeringValue = Constants.Vision.kP * horizontalError + Constants.Vision.kI * sumOfErrors
                        + Constants.Vision.kD * recentError + Constants.Vision.kS;
            }
        }
        leftSteeringAngle += adjustedSteeringValue;
        rightSteeringAngle -= adjustedSteeringValue;

        drive.setOutputPercent(leftSteeringAngle, rightSteeringAngle);
        lastError = horizontalError;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
