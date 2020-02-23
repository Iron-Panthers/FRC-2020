/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public final TalonFX armLeft;
    public final TalonFX armRight;
    private final DigitalInput forwardLimitSwitch;
    private final DigitalInput reverseLimitSwitch;
    private final LimelightWrapper limelight;
	public int targetHeight;

    /**
     * Creates a new Arm. For limits, forward refers to the front, in which the arm
     * is all the way down and ready for intaking. Reverse refers to the back, in
     * which the arm is all the way up at the maximum angle for shooting.
     * <p>
     * As usual, only one of these should ever be constructed.
     */
    public Arm(LimelightWrapper limelight) {
        this.limelight = limelight;
        armLeft = new TalonFX(Constants.Arm.kLeftMotorId);
        armRight = new TalonFX(Constants.Arm.kRightMotorId);
        armLeft.setSensorPhase(false); // Up is positive
        armLeft.setInverted(true);
        armRight.setInverted(InvertType.OpposeMaster);
        armLeft.setNeutralMode(NeutralMode.Brake);
        armRight.setNeutralMode(NeutralMode.Brake);
        armRight.follow(armLeft);

        // Current Limits and Power Limits
        armLeft.configClosedLoopPeakOutput(Constants.Arm.kPIDIdx, Constants.Arm.kClosedLoopPeakOutput);
        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
                Constants.Arm.kCurrentLimit, Constants.Arm.kCurrentLimit, 0);
        armLeft.configSupplyCurrentLimit(currentConfig);
        armLeft.configClosedloopRamp(Constants.Arm.kRampRate);

        // Limit switches
        // Forward needs to be the highest positive value, so the high position
        // Reverse needs to be the lowest value, so the ground position
        forwardLimitSwitch = new DigitalInput(Constants.Arm.kHighLimitSwitchPort);
        reverseLimitSwitch = new DigitalInput(Constants.Arm.kGroundLimitSwitchPort);
        armLeft.configForwardSoftLimitEnable(true);
        armLeft.configReverseSoftLimitEnable(true);
        armLeft.configForwardSoftLimitThreshold(Constants.Arm.kTopSoftLimit);
        armLeft.configReverseSoftLimitThreshold(Constants.Arm.kBottomSoftLimit);

        configPIDF(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD, Constants.Arm.kF, Constants.Arm.kPIDIdx);
    }

    public void setPower(double power) {
        armLeft.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setVoltage(double voltage) {
        armLeft.set(TalonFXControlMode.PercentOutput, voltage / RobotController.getBatteryVoltage());
    }

    public void stop() {
        setPower(0);
    }

    public void configPIDF(double p, double i, double d, double f, int idx) {
        armLeft.config_kP(idx, p);
        armLeft.config_kI(idx, i);
        armLeft.config_kD(idx, d);
        armLeft.config_kF(idx, f);
    }

    /**
     * Set velocity of the arm, will be used in tuning PID for velocity to get
     * values for MotionMagic
     */
    public void setVelocity(double nativeUnits) {
        armLeft.set(TalonFXControlMode.Velocity, nativeUnits);
    }

    public void setPosition(int target) {
        armLeft.set(TalonFXControlMode.Position, target);
    }

    /**
     * Motion Magic may not be needed, position is probably good enough
     */
    public void setMotionMagicPosition(int target) {
        armLeft.set(TalonFXControlMode.MotionMagic, target);
    }

    public double getFeedForward() {
        double scaledAngle = Math.cos(Math.toRadians(getAngle()));
        // double f = Constants.Arm.kHorizontalHoldVoltage * scaledAngle; // Voltage
        // based
        double f = Constants.Arm.kHorizontalHoldOutput * scaledAngle;
        // PercentOutput with feedforward to avoid oscillation which wears down the
        // gears
        return f;
    }

    public double getHeight() {
        return Constants.Vision.kPivotToLL * Math.sin((getAngle() + Constants.Vision.kPivotToLLAngle) * Math.PI / 180)
                + Constants.Vision.kGroundToPivotInches;
    }

    public double getPivotToLLHorizontleD(double angle) {
        return (getHeight() - Constants.Vision.kGroundToPivotInches) / Math.tan((angle + Constants.Vision.kPivotToLLAngle) * Math.PI / 180);
    }
    // public double getDiagonalDistance(){
    // return Math.sqrt(Math.pow(getHorizontalDistance(), 2) +
    // Math.pow(Constants.Vision.kGroundToTargetInches - getHeight(), 2));
    // }

    public double getAngle() {
        double currentAngle = (armLeft.getSelectedSensorPosition() * Constants.Arm.encoderToAngle)
                + Constants.Arm.kArmAngleOffset;

        return currentAngle;
    }

    public double getLlOffset() {
        return getPivotToLLHorizontleD(getAngle()) - getPivotToLLHorizontleD(getAngle() + limelight.getTableY());
    }

    public double getHAnlge() {
        return 90 - Constants.Vision.kMountToLLAngleDeg - getAngle() + limelight.getTableY();
    }

    public double getHorizontalDistance() {
        limelight.periodic();
        double HorizontalDistance = 0;
        double llAngleDeg = Constants.Vision.kMountToLLAngleDeg;
        double offset = getPivotToLLHorizontleD(getAngle())
                - getPivotToLLHorizontleD(getAngle() + limelight.getTableY());
        double hAngle = 90 - llAngleDeg - getAngle() + limelight.getTableY();
        if (limelight.getTableY() >= 0) {
            HorizontalDistance = (Constants.Vision.kGroundToTargetInches - getHeight())
                / (Math.tan(Math.toRadians(hAngle))) - offset;
        } else {
            HorizontalDistance = (Constants.Vision.kGroundToTargetInches - getHeight())
                / (Math.tan(Math.toRadians(hAngle)));
        }
        
        return HorizontalDistance;
    }//inches

    public int getPosition() {
        return armLeft.getSelectedSensorPosition();
    }

    public int getVelocity() {
        return armLeft.getSelectedSensorVelocity();
    }

    public double getOutputVoltage() {
        return armLeft.getMotorOutputVoltage();
    }

    public double getOutputCurrent() {
        return armLeft.getStatorCurrent();
    }

    public void setZero() {
        armLeft.setSelectedSensorPosition(0);
    }

    public boolean getGroundLimitPressed() {
        return !reverseLimitSwitch.get();
    }

    public boolean getHighLimitPressed() {
        return !forwardLimitSwitch.get();
    }

    @Override
    public void periodic() {
        if (getGroundLimitPressed()) {
            setZero();
        } else if (getHighLimitPressed()) {
            armLeft.setSelectedSensorPosition(Constants.Arm.kTopPositionNativeUnits);
        } 
        SmartDashboard.putBoolean("High Limit", getHighLimitPressed());
        SmartDashboard.putBoolean("Ground Limit", getGroundLimitPressed());
        SmartDashboard.putNumber("Arm Position", getPosition());
        SmartDashboard.putNumber("Arm Output Voltage", getOutputVoltage());
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("HORE", getHorizontalDistance());

        // // If within the slow threshold, limit output to scaled regular output
        // if (getPosition() < Constants.Arm.BOTTOM_SLOW_LIMIT || getPosition() >
        // Constants.Arm.TOP_SLOW_LIMIT) {
        // armLeft.configClosedLoopPeakOutput(Constants.Arm.ARM_POSITION_PID_SLOT,
        // Constants.Arm.SLOW_ARM_PID_OUTPUT);
        // }
        // // If out of the slow threshold, reset output to correct value
        // else {
        // armLeft.configClosedLoopPeakOutput(Constants.Arm.ARM_POSITION_PID_SLOT,
        // Constants.Arm.MAX_ARM_PID_OUTPUT);
        // }
    }
}