/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private final TalonFX shooter1;
  private final TalonFX shooter2;
  private final TalonFX shooter3;

  public Shooter() {
    shooter1 = new TalonFX(Ports.shooter1);
    shooter2 = new TalonFX(Ports.shooter2);
    shooter3 = new TalonFX(Ports.shooter3);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
