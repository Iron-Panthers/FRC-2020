/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {

  public static NetworkTable table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;
  public static NetworkTableEntry tv;
  public static float x;
  public static float y;
  public static float v;

  public TurnToTarget() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
    x = (float) tx.getDouble(0.0);
    y = (float) ty.getDouble(0.0);
    v = (float) tv.getDouble(0.0);
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
    float horizontalError = -x; // Horizontal error from limelight to target
    float adjustedSteeringValue = 0.0f; // Calculated amount of degrees to steer

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
