/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.RobotContainer;
import com.ironpanthers.frc2020.subsystems.Arm;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	public static TalonFX armLeft;
	public static TalonFX armRight;
	public Encoder encoder;
	private DigitalInput forwardLimitSwitch;
	private DigitalInput reverseLimitSwitch;
	private RobotContainer robotContainer;
	private int target;
	private double horizontalHoldOuput; //TODO figure out this value by testing

	/**
	 * Creates a new Arm.
	 */
	public Arm() {
		armLeft = new TalonFX(Constants.Arm.ARM_LEFT_PORT);
		armRight = new TalonFX(Constants.Arm.ARM_RIGHT_PORT);
		armLeft.setSensorPhase(false);
		armLeft.setInverted(Constants.Arm.IS_LEFT_ARM_INVERTED);
		armRight.setInverted(Constants.Arm.IS_RIGHT_ARM_INVERTED);
		armLeft.setNeutralMode(NeutralMode.Brake);
		armRight.setNeutralMode(NeutralMode.Brake);
		armRight.follow(armLeft);
		forwardLimitSwitch = new DigitalInput(Constants.Arm.FORWARD_LIMIT_SWTICH_PORT);
		reverseLimitSwitch = new DigitalInput(Constants.Arm.REVERSE_LIMIT_SWITCH_PORT);
		encoder.setDistancePerPulse(1./2048.);
	}

	public void setPower(double power) {
		armLeft.set(TalonFXControlMode.PercentOutput, power);
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
	 * Set velocity of the arm, will be used in tuning PID for velocity to get values for MotionMagic
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

	public void handleAcceleration(int sensorUnitAccel) {
		if(getCurrentAngle() < target){
			armLeft.configMotionAcceleration(sensorUnitAccel); //TODO find out what parameter to put here
			armRight.configMotionCruiseVelocity(sensorUnitAccel);
		}
	}

	public void setTargetPosition(int target) {
		this.target = target;
		SmartDashboard.putNumber("Arm Target", target);
	}

	public void setFeedForward() { 
		double scaledAngle = Math.cos(Math.toRadians(getCurrentAngle()));
		armLeft.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, Constants.Arm.MAX_FF * scaledAngle);
		SmartDashboard.putNumber("Arbitrary Feedforward", horizontalHoldOuput * scaledAngle);
	}

	public double getCurrentAngle() { 
		double currentAngle = (armLeft.getSelectedSensorPosition() * Constants.Arm.TICKS_TO_DEGREES) + Constants.Arm.ARM_ANGLE_OFFSET;
		SmartDashboard.putNumber("Current angle", currentAngle);
		//returns angle in degrees
		return currentAngle; 
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

	@Override
	public void periodic() {
		if (encoder.getDistance() < Constants.Arm.ARM_TICKS && encoder.getDistance() > Constants.Arm.ARM_TICKS-1000) {
			Arm.armLeft.set(TalonFXControlMode.PercentOutput, 0.001*(Constants.Arm.ARM_TICKS - encoder.getDistance()));
		}
		else if (encoder.getDistance() >0  && encoder.getDistance() < 1000) {
			Arm.armLeft.set(TalonFXControlMode.PercentOutput, 0.001*(Constants.Arm.ARM_TICKS - encoder.getDistance()));
		}	
		SmartDashboard.putNumber("Shooter Current", robotContainer.shooter.shooter1.getStatorCurrent());
		
		// This method will be called once per scheduler run
	}
}