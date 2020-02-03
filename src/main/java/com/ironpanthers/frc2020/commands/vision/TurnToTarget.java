/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {

  public double x;
  public double y;
  public double v;
  double[] totalErrors;
  double lastError;
  double sumOfErrors;

  public TurnToTarget() {    
    x = Robot.visionWrapper.getX();
    y = Robot.visionWrapper.getY();
    v = Robot.visionWrapper.getV();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    totalErrors = new double[100];
    lastError = 0;
    sumOfErrors = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSteeringAngle = 0.0; // Amount of degrees to steer left
    double rightSteeringAngle = 0.0; // Amount of degrees to steer right
    double horizontalError = -x; // Horizontal error from limelight to target
    double adjustedSteeringValue = 0.0; // Calculated amount of degrees to steer
    double recentError = 0.0;

    for(int i = 98; i>=0; i--){
      totalErrors[i+1] = totalErrors[i];
    }
    totalErrors[0] = horizontalError;

    for(int i = 0; i<totalErrors.length; i++){
      sumOfErrors += totalErrors[i];
    }

    recentError = horizontalError-lastError;

    if (v == 0.0) {
      adjustedSteeringValue = 0.3;
    }
    else {
      if (x > 0.0) {
        adjustedSteeringValue = Constants.Vision.Kp * horizontalError + Constants.Vision.Ki * sumOfErrors + Constants.Vision.Kd * recentError - Constants.Vision.MINIMUM_POWER;
      } else if (x < 0.0) {
        adjustedSteeringValue = Constants.Vision.Kp * horizontalError + Constants.Vision.Ki * sumOfErrors + Constants.Vision.Kd * recentError + Constants.Vision.MINIMUM_POWER;
      }
    }

    leftSteeringAngle += adjustedSteeringValue;
    rightSteeringAngle -= adjustedSteeringValue;
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
