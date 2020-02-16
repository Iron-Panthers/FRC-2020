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

public class VisionTesting extends CommandBase {
  /**
   * Creates a new HorizontalDistance.
   */
  LimelightWrapper lWrapper;
  Arm arm;
  double HorizontalDistance;

  public VisionTesting(LimelightWrapper limelightWrapper, Arm arm) {
    lWrapper = limelightWrapper;
    this.arm = arm;
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
    lWrapper.turnOnLight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lWrapper.turnOffLight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
