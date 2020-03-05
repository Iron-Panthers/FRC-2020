/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.util.LimelightWrapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX armLeft;
    private final TalonFX armRight;
    private final CANCoder canCoder;
    // private final DigitalInput forwardLimitSwitch;
    private final DigitalInput reverseLimitSwitch;
    private final LimelightWrapper limelight;
    private final Solenoid diskBrake;
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
        canCoder = new CANCoder(Constants.Arm.kCANCoderId);
        diskBrake = new Solenoid(Constants.Arm.kBrakePort);
		canCoder.configFactoryDefault();
		canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.configSensorDirection(Constants.Arm.kSensorPhase);
        canCoder.configMagnetOffset(Constants.Arm.kCANCoderOffset);
		calibrateCANCoder();
        canCoder.configFeedbackCoefficient(Constants.Arm.kCanCoderCoefficient, "deg", SensorTimeBase.PerSecond); // Degrees per second, output in degrees
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        armLeft.configRemoteFeedbackFilter(canCoder, Constants.Arm.kRemoteSensorSlot);
        armLeft.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0); // Should be the same number as Constants.Arm.kRemoteSensorSlot
        armLeft.setSensorPhase(Constants.Arm.kSensorPhase); // Up is positive
        armLeft.setInverted(Constants.Arm.kSensorPhase); // Should be the same
        
        armLeft.setNeutralMode(NeutralMode.Brake);
        armRight.setNeutralMode(NeutralMode.Brake);
        armRight.follow(armLeft);
        armRight.setInverted(InvertType.OpposeMaster);

        // Current Limits and Power Limits
        armLeft.configClosedLoopPeakOutput(Constants.Arm.kPIDIdx, Constants.Arm.kClosedLoopPeakOutput);
        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
                Constants.Arm.kCurrentLimit, Constants.Arm.kCurrentLimit, 0);
        armLeft.configSupplyCurrentLimit(currentConfig);
        armLeft.configClosedloopRamp(Constants.Arm.kRampRate);

        // Limit switches
        // Forward needs to be the highest positive value, so the high position
        // Reverse needs to be the lowest value, so the ground position
        // forwardLimitSwitch = new DigitalInput(Constants.Arm.kHighLimitSwitchPort);
        reverseLimitSwitch = new DigitalInput(Constants.Arm.kGroundLimitSwitchPort);
        armLeft.configForwardSoftLimitEnable(false);
        armLeft.configReverseSoftLimitEnable(false);
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
    public void setVelocity(double degreesPerSecond) {
        armLeft.set(TalonFXControlMode.Velocity, degreesPerSecond);
    }

	/**
	 * Set position using degrees
	 * @param degrees in degrees
	 */
	public void setPosition(double degrees) {
		armLeft.set(TalonFXControlMode.Position, degrees / Constants.Arm.kCanCoderCoefficient);
	}

    public double getHeight() {
        return Constants.Vision.kPivotToLL * Math.sin((getAngle() + Constants.Vision.kPivotToLLAngle) * Math.PI / 180)
                + Constants.Vision.kGroundToPivotInches;
    }

    public double getHeight2() {
        return Math.sqrt(Math.pow(Constants.Vision.kPivotToLL, 2) - Math.pow(getPivotToLLHorizontleD(getAngle()), 2))
                + Constants.Vision.kGroundToPivotInches;
    }
    public double getAngleTrig() {
        return Math.toDegrees(Math.asin((getHeight2()-10)/Constants.Vision.kPivotToLL)) - 30;
    }

    public double getPivotToLLHorizontleD(double angle) {
        return Constants.Vision.kPivotToLL * Math.cos((angle + Constants.Vision.kPivotToLLAngle) * Math.PI / 180);
    }
    // public double getDiagonalDistance(){
    // return Math.sqrt(Math.pow(getHorizontalDistance(), 2) +
    // Math.pow(Constants.Vision.kGroundToTargetInches - getHeight(), 2));
    // }

    public double getAngle() {
        double currentAngle = canCoder.getAbsolutePosition();
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
        double offset = getPivotToLLHorizontleD(getAngle())
                - getPivotToLLHorizontleD(getAngle() + limelight.getTableY());
        HorizontalDistance = (Constants.Vision.kGroundToTargetInches - getHeight2())
                / (Math.tan(Math.toRadians(getHAnlge()))) + offset;
        return HorizontalDistance;
    }

    public double getDiagonalDistance() {
        return Math.sqrt(Math.pow(Constants.Vision.kGroundToTargetInches - getHeight(), 2) + Math.pow(getHorizontalDistance(), 2));
    }

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

    public boolean getGroundLimitPressed() {
        return !reverseLimitSwitch.get();
    }

    public boolean getHighLimitPressed() {
        return !reverseLimitSwitch.get() && canCoder.getAbsolutePosition() > Constants.Arm.kUseTopLimitRange;
    }

    public void calibrateCANCoder() {
        // Set CANCoder position so it aligns with the absolute position
        canCoder.setPositionToAbsolute();
    }

    public void engageBrake() { // Needs testing
        diskBrake.set(false);
    }

    public void releaseBrake() { // Needs testing
        diskBrake.set(true);
    }

    public void setZero() {
        canCoder.setPosition(0);
    }

    public void setSensorPosition(double degrees) {
        canCoder.setPosition(degrees);
    }

    @Override
    public void periodic() {
        if (getGroundLimitPressed()) {
            // setZero();
        } /*else if (getHighLimitPressed()) {
            setSensorPosition(Constants.Arm.kTopPositionDegrees);
        }*/
        // SmartDashboard.putBoolean("Ground Limit", getGroundLimitPressed());
		SmartDashboard.putNumber("Arm Angle", getAngle());
		SmartDashboard.putNumber("Arm Position", getPosition());
		SmartDashboard.putNumber("Arm Voltage", getOutputVoltage());
        // SmartDashboard.putNumber("Arm Height", getHeight2());
        SmartDashboard.putNumber("getHorizontalDistance", getHorizontalDistance());
        // SmartDashboard.putNumber("offset", getLlOffset());
        // SmartDashboard.putNumber("hAngle", getHAnlge());
        // SmartDashboard.putNumber("GetAngleTrig", getAngleTrig());
        // SmartDashboard.putNumber("Height Difference", Constants.Vision.kGroundToTargetInches - getHeight());


    }
}