/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
  /**
   * Creates a new TurnToTarget.
   */
  public TurnToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    float leftSteeringAngle = 0.0f; // Amount of degrees to steer left
    float rightSteeringAngle = 0.0f; // Amount of degrees to steer right
    float horizontalError = -tx; // Horizontal error from limelight to target
    float adjustedSteeringValue = 0.0f; // Calculated amount of degrees to steer

    if (tv == 0.0f) {
      adjustedSteeringValue = 0.3f;
    }
    else {
      if (tx > 1.0) {
        adjustedSteeringValue = Constants.Kp * horizontalError - Constants.MINIMUM_POWER;
      } else if (tx < 1.0) {
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
