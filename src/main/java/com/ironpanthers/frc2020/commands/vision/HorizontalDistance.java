/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.vision;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HorizontalDistance extends CommandBase {
  /**
   * Creates a new HorizontalDistance.
   */
  LimelightWrapper lWrapper;
  Arm arm;
  double HorizontalDistance;

  public HorizontalDistance(LimelightWrapper limelightWrapper, Arm arm) {
    lWrapper = limelightWrapper;
    this.arm = arm;
    SmartDashboard.putNumber("Limelight mounting angle", Constants.Vision.kMountToLLAngleDeg);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lWrapper.periodic();
    double llAngleDeg = SmartDashboard.getNumber("Limelight mounting angle", Constants.Vision.kMountToLLAngleDeg);
    double offset = arm.getPivotToLLHorizontleD(arm.getAngle()) - arm.getPivotToLLHorizontleD(arm.getAngle() + lWrapper.getTableY());
    double hAngle = 90 - llAngleDeg - arm.getAngle() + lWrapper.getTableY();
    double HorizontalDistance = (Constants.Vision.kGroundToTargetInches - arm.getHeight())/(Math.tan(Math.toRadians(hAngle))) - offset;

    // SmartDashboard.putNumber("current angle", arm.getAngle());

    // SmartDashboard.putNumber("a1 in degrees",
    //     (Math.toDegrees(Math.atan(
    //         (Constants.Vision.kGroundToTargetInches - arm.getHeight()) / (105.5 + Constants.Vision.kGroundToPivotInches))))
    //         - lWrapper.getTableY());

    // SmartDashboard.putNumber("Aaron Method",
    //     Math.atan(-1 / Math.tan(arm.getAngle())) + (90 - Constants.Vision.kMountToLLAngleDeg) + lWrapper.getTableY());
    SmartDashboard.putNumber("Offset", offset);
    SmartDashboard.putNumber("HMethod", 90 - llAngleDeg - arm.getAngle() + lWrapper.getTableY());

    SmartDashboard.putNumber("Hotizontal Distance", HorizontalDistance);
    
    // the 30 is a placeholder for distance
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
