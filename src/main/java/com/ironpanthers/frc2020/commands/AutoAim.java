// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.subsystems.Arm;

public class AutoAim extends CommandBase {
  private PIDController steeringController;
  private PIDController armController;
  private Drive drive;
  private Arm arm;
  private LimelightWrapper limelight;
  private ShuffleboardTab limeboard;

  /** Creates a new AutoAim. */
  public AutoAim(Drive drive, Arm arm, LimelightWrapper limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    steeringController = new PIDController(.022, 0, .002);
    armController = new PIDController(0.01, 0, 0);
    this.drive = drive;
    this.arm = arm;
    this.limelight = limelight;
    this.limeboard = Shuffleboard.getTab("steering");
    limeboard.add("steering pid", steeringController);
    limeboard.add("arm pid", armController);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setOutputPercent(0.0, 0.0);
    new ArmToTarget(arm, 65).schedule();
    steeringController.setSetpoint(0);
    armController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = limelight.getTableX();
    double yOffset = limelight.getTableY();

    double driveGoal = steeringController.calculate(xOffset);
    drive.setOutputPercent(driveGoal, -driveGoal);

    double armGoal = armController.calculate(yOffset);
    arm.releaseBrake();
    arm.setPower(armGoal);
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
