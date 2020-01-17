/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.Ports;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorBelt extends SubsystemBase {
  public static TalonFX motor;
  public static DigitalInput input;
  public static Encoder encoder;
  
  public ConveyorBelt() {
    input = new DigitalInput(Ports.DIGITAL_INPUT_PORT);
    motor = new TalonFX(Ports.CONVEYOR_BELT_MOTOR_PORT);
    encoder = new Encoder(0, 0);
    encoder.setDistancePerPulse(1./25.); //1 inch per encoder rotation, 25 pulses per encoder rotation
  }

  public static void moveOneBall() {
    while (encoder.getDistance() < Constants.POWER_CELL_DIAMETER) {
      motor.set(ControlMode.PercentOutput, Constants.CONVEYOR_BELT_MOTOR_POWER);
    }
    motor.set(ControlMode.PercentOutput, 0);
    encoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
