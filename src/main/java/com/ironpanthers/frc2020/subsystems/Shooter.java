/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ironpanthers.frc2020.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  public final TalonFX shooter1;
  public final TalonFX shooter2;
  public final TalonSRX shooter3;

  public Shooter() {
    shooter1 = new TalonFX(Ports.shooter1);
    shooter2 = new TalonFX(Ports.shooter2);
    shooter3 = new TalonSRX(Ports.shooter3);

    shooter2.follow(shooter1);
    shooter3.follow(shooter1);
  }

  public void shootWithSpeed(double speed) {
    shooter1.set(ControlMode.PercentOutput, speed);
  }

  public void stopShooter() {
    shooter1.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
