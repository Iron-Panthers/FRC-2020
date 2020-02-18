/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmInterpolation extends CommandBase {
  private final Shooter shooter;
  private int height;
  private ConveyorBelt conveyorBelt;
  private LimelightWrapper lWrapper;
  private Arm arm;

  /**
   * Creates a new ShootAtVelocity.
   */
  public ArmInterpolation(Shooter shooter, ConveyorBelt conveyorBelt, LimelightWrapper lWrapper, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyorBelt = conveyorBelt;
    this.lWrapper = lWrapper;
    this.arm = arm;
    this.shooter = shooter;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (conveyorBelt.ballsHeld <= 0) { // we have no balls or mis-indexed TODO reconsider
      lWrapper.turnOffLight();
      cancel();
    }

    height = shooter.interpolateY(arm.getHorizontalDistance(), shooter.armPosTable);
    arm.setPosition(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Horizontal Distance", arm.getHorizontalDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.epsilonEquals(arm.getPosition(), height, Constants.Arm.kPositionErrorTolerance);
  }
}
