/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.util;

/**
 * Add your docs here.
 */
public class ThreeDimensionalSegment {
    public double xcomp; //x component of segment
    public double ycomp;
    public double zcomp;
    public double magnitude;

    /**
     * a 3d vector-like object created using an estimated magnitude and the two known xy points seen in the image
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param magnitude
     */
    public ThreeDimensionalSegment(double x1, double y1, double x2, double y2, double magnitude) {
        xcomp = Math.abs(x1 - x2); 
        ycomp = Math.abs(y1 - y2);
        zcomp = Math.sqrt(magnitude * magnitude - xcomp * xcomp - ycomp * ycomp);
        this.magnitude = magnitude;
    }

    /**
     * calculates the new z component of a vector assuming x and the magnitude stay the same but y goes to zero
     * @return
     */
    public double calcHorizontalZ() {
        return Math.sqrt(zcomp * zcomp + ycomp * ycomp);
    }

    /**
     * gets estimated angle offset from center that the hole is given estimated distance from outer goal to hole based on distance away from scoring location
     * @return
     */
    public double getOffsetAngle(double outerGoalToHole, double distanceToOuterGoal) {
        
        //law of cosines with known side lengths
        return (180/Math.PI) * Math.acos( ( Math.pow(distanceToOuterGoal, 2) + Math.pow(getDistanceToHole(outerGoalToHole, distanceToOuterGoal), 2) - Math.pow(outerGoalToHole, 2) ) 
        / (2 * distanceToOuterGoal * getDistanceToHole(outerGoalToHole, distanceToOuterGoal) ) );

    }

    public double getDistanceToHole(double outerGoalToHole, double distanceToOuterGoal) {

        double refAngle = Math.atan( xcomp / calcHorizontalZ() );
        double goalToHoleXComp = Math.cos(refAngle) * outerGoalToHole;
        double goalToHoleZComp = Math.sin(refAngle) * outerGoalToHole;

        return Math.sqrt( Math.pow(goalToHoleZComp + distanceToOuterGoal, 2) + Math.pow(goalToHoleXComp, 2) );
    }

}
