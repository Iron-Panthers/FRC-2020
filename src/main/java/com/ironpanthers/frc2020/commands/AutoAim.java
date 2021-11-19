// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ironpanthers.frc2020.subsystems.Drive;

public class AutoAim extends CommandBase {
  private PIDController steeringController;
  private Drive drive;
  private LimelightWrapper limelight;

  /** Creates a new AutoAim. */
  public AutoAim(Drive drive, LimelightWrapper limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    steeringController = new PIDController(.2, 0, 0);
    this.drive = drive;
    this.limelight = limelight;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setOutputPercent(0.0, 0.0);
    steeringController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = limelight.getTableX();
    // int yOffset = limelight.getTableY();

    double goal = steeringController.calculate(xOffset);

    drive.setOutputPercent(goal, -goal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return steeringController.getPositionError() < .1;
  }
}
