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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  public static TalonFX controlPanelMotor;

  public ControlPanel() {
    controlPanelMotor = new TalonFX(Constants.CONTROL_PANEL_MOTOR_PORT);
  }

  public static void colorRotation(int rotations, int direction) { // Direction: -1 for left, 1 for right
    controlPanelMotor.setSelectedSensorPosition(0);
    double distance = rotations * Constants.ENCODER_TICKS_PER_INCH * 12.5; // Each rotation is 12.5 inches apart
    if(controlPanelMotor.getSelectedSensorPosition() < distance){
      controlPanelMotor.set(ControlMode.PercentOutput, direction * Constants.CONTROL_PANEL_MOTOR_SPEED);
    } else {
      controlPanelMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
