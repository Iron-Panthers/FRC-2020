/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.RobotContainer;
import com.ironpanthers.frc2020.subsystems.Arm;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	public static TalonFX armLeft;
	public static TalonFX armRight;
	private DigitalInput forwardLimitSwitch;
	private DigitalInput reverseLimitSwitch;
	private RobotContainer robotContainer;

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

<<<<<<< HEAD
=======
	public void setFeedForward(int target){ //TODO figure out how to get target position
		double scaledAngle = Math.cos(Math.toRadians(getCurrentAngle()));
		armLeft.set(ControlMode.MotionMagic, target, DemandType.ArbitraryFeedForward, Constants.Arm.MAX_FF * scaledAngle);
		SmartDashboard.putNumber("Arbitrary Feedforward", Constants.Arm.MAX_FF * scaledAngle);
	}

	//return angle in degrees
	public double getCurrentAngle(){
		double currentAngle = (armLeft.getSelectedSensorPosition() * Constants.Arm.TICKS_TO_DEGREES) + Constants.Arm.ARM_ANGLE_OFFSET;
		SmartDashboard.putNumber("Current angle", currentAngle);
		return currentAngle;
	}
	
>>>>>>> temporary-combined-mechanisms
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
		int output = (int) RobotContainer.joystick.getY();
		if (forwardLimitSwitch.get()) {
			output = Math.min(-output, 0);
		}
		else if (reverseLimitSwitch.get()) {
			output = Math.max(-output, 0);
		}	
		Arm.armLeft.set(TalonFXControlMode.PercentOutput, output*Constants.Arm.LIMIT_SWITCH_P);
		Arm.armRight.set(TalonFXControlMode.PercentOutput, output*Constants.Arm.LIMIT_SWITCH_P);
		SmartDashboard.putNumber("Shooter Current", robotContainer.shooter.shooter1.getStatorCurrent());
		// This method will be called once per scheduler run
	}
}