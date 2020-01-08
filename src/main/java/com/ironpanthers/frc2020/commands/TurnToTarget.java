/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.util.VisionWrapper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {

  public static VisionWrapper limelight;

  public double x;
  public double y;
  public double v;

  public TurnToTarget() {    
    x = limelight.getX();
    y = limelight.getY();
    v = limelight.getV();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double leftSteeringAngle = 0.0; // Amount of degrees to steer left
    double rightSteeringAngle = 0.0; // Amount of degrees to steer right
    double horizontalError = -x; // Horizontal error from limelight to target
    double adjustedSteeringValue = 0.0; // Calculated amount of degrees to steer

    if (v == 0.0) {
      adjustedSteeringValue = 0.3f;
    }
    else {
      if (x > 0.0) {
        adjustedSteeringValue = Constants.Kp * horizontalError - Constants.MINIMUM_POWER;
      } else if (x < 0.0) {
        adjustedSteeringValue = Constants.Kp * horizontalError + Constants.MINIMUM_POWER;
      }
    }
    
    leftSteeringAngle += adjustedSteeringValue;
    rightSteeringAngle -= adjustedSteeringValue;
    
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
