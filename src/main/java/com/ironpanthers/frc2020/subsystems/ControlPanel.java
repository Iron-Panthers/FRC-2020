/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  public static TalonFX controlPanelMotor;

  public ControlPanel() {
    controlPanelMotor = new TalonFX(Constants.CONTROL_PANEL_MOTOR_PORT);
  }

  public static void colorRotation(int rotations, int direction) { // Direction: -1 for counter-clockwise, 1 for clockwise
    controlPanelMotor.setSelectedSensorPosition(0);
    double distance = rotations * Constants.ENCODER_TICKS_PER_INCH * 12.5; // Each rotation is 12.5 inches apart
    if(controlPanelMotor.getSelectedSensorPosition() < distance){
      controlPanelMotor.set(ControlMode.PercentOutput, direction * Constants.CONTROL_PANEL_MOTOR_SPEED);
    } else {
      controlPanelMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public static void positionControl(char initColor) {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    List<Character> colors = new ArrayList<Character>();
    colors.add('B');
    colors.add('G');
    colors.add('R');
    colors.add('Y');
    if(gameData.length() > 0) {
      int placement = colors.indexOf(gameData.charAt(0)) - colors.indexOf(initColor); // Distance between colors
      if(placement < 0) { 
        if(placement == 3) { // If the color is 3 rotations away one direction, make it turn 1 rotation the opposite direction
          colorRotation(1, 1);
        } else {
          colorRotation(Math.abs(placement), -1);
        }
      } else if(placement > 0) {
        if(placement == 3) {
          colorRotation(1, -1);
        } else {
          colorRotation(Math.abs(placement), 1);
        }
      }
    } else {
      //Code for no data received yet
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
