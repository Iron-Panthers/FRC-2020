/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.ironpanthers.frc2020.util;

import com.fasterxml.jackson.core.util.TextBuffer;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Alignment {

  public float calculateDistance() {
    float d = (Constants.h2 - Constants.h1) / tan(Constants.a1 + ty); // ty: vertical offset angle in degrees
    return d;
  }

}