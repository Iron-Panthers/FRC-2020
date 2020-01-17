/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Ports;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorBelt extends SubsystemBase {
  public static TalonFX motor;
  public static DigitalInput input;
  
  public ConveyorBelt() {
    input = new DigitalInput(Ports.DIGITAL_INPUT_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
