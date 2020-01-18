/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorBelt extends SubsystemBase {
  public static TalonFX conveyorMotor;
  private static DigitalInput input;
  public static CANCoder encoder;
  public static int ballsHeld;
  
  public ConveyorBelt() {
    input = new DigitalInput(Constants.Conveyor.DIGITAL_INPUT_PORT);
    conveyorMotor = new TalonFX(Constants.Conveyor.CONVEYOR_BELT_MOTOR_PORT);
    encoder = new CANCoder(Constants.Conveyor.CANCODER_PORT);
    ballsHeld = 0;
  }
  public boolean getBannerSensor() {
    return input.get();
  }
  public void setPower(double power) {
    conveyorMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public boolean conveyorFull() {
    return ballsHeld >= 5;
  }


  // public static void moveOneBall() {
  //   while (encoder.getDistance() < Constants.POWER_CELL_DIAMETER) {
  //     motor.set(ControlMode.PercentOutput, Constants.CONVEYOR_BELT_MOTOR_POWER);
  //   }
  //   motor.set(ControlMode.PercentOutput, 0);
  //   encoder.reset();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
