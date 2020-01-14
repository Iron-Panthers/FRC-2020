/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  TalonFX shooter1 = new TalonFX(Constants.Shooter.SHOOTER_ONE_PORT);
  TalonFX shooter2 = new TalonFX(Constants.Shooter.SHOOTER_TWO_PORT);
  TalonFX shooter3 = new TalonFX(Constants.Shooter.SHOOTER_THREE_PORT);

  public Shooter() {

  }

  public void setSpeed(double speed) {
    shooter1.set(TalonFXControlMode.PercentOutput, speed);
    shooter2.set(TalonFXControlMode.PercentOutput, speed);
    shooter3.set(TalonFXControlMode.PercentOutput, speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
