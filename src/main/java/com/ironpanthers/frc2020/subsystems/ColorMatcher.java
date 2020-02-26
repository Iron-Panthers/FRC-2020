/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import java.util.List;

import com.ironpanthers.frc2020.Constants;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 * <p>
 * It is based on the example which can be found here
 * https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java.
 */
public class ColorMatcher extends SubsystemBase {
    /**
     * Creates a new ColorMatcher.
     */
    public ColorMatcher() {
        for (Color c : List.of(kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget)) {
            colorMatcher.addColorMatch(c);
        }
        colorMatcher.setConfidenceThreshold(.8);
    }

    // Should match connection type for color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This
     * can be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * Colors can be re-calibrated, these are just ones REV provides.
     */
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public String getColor() {
        final var detectedColor = colorSensor.getColor();

        final String colorAsString;
        final var matchResult = colorMatcher.matchClosestColor(detectedColor);

        if (matchResult.color == kBlueTarget) {
            colorAsString = "Blue";
            System.out.println(Constants.ROTATIONS + .5);
        } else if (matchResult.color == kRedTarget) {
            colorAsString = "Red";
        } else if (matchResult.color == kGreenTarget) {
            colorAsString = "Green";
        } else if (matchResult.color == kYellowTarget) {
            colorAsString = "Yellow";
        } else {
            colorAsString = "Unknown";
        }

        final var rgbArray = new double[] { detectedColor.red, detectedColor.blue, detectedColor.green };
        SmartDashboard.putNumberArray("ColorMatcher/RGB", rgbArray);
        SmartDashboard.putString("ColorMatcher/DetectedColor", colorAsString);
        SmartDashboard.putNumber("ColorMatcher/Confidence", matchResult.confidence);
        return colorAsString;
    }

    @Override
    public void periodic() {
        /**
         * The method GetColor() returns a normalized color value from the sensor and
         * can be useful if outputting the color to an RGB LED or similar. To read the
         * raw color, use GetRawColor().
         * 
         * The color sensor works best when within a few inches from an object in well
         * lit conditions (the built in LED is a big help here!). The farther an object
         * is the more light from the surroundings will bleed into the measurements and
         * make it difficult to accurately determine its color.
         */
        final var colorAsString = getColor();
        SmartDashboard.putString("ColorMatcher/DetectedColor", colorAsString);
    }
}