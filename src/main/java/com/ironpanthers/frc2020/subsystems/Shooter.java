/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ironpanthers.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final TalonFX shooter3;
    private final TalonFX intakeMotor;
    public int velocity;
    // TODO: Tune
    private final double[] distanceTable = { 0, 120.0, 240.0, 408.0 }; // Inches
	public final int[] velocityTable = {Constants.Shooter.kCloseVelocity, Constants.Shooter.kInitiationVelocity, Constants.Shooter.kFarVelocity}; // Units/100ms
	public final int[] armPosTable = {Constants.Arm.kCloseShotHeightNativeUnits, Constants.Arm.kInitiationLineHeight, Constants.Arm.kFarShotHeightNativeUnits};

    public Shooter() {
        shooter1 = new TalonFX(Constants.Shooter.kShooter1Id);
        shooter2 = new TalonFX(Constants.Shooter.kShooter2Id);
        shooter3 = new TalonFX(Constants.Shooter.kShooter3Id);
        intakeMotor = new TalonFX(Constants.Conveyor.kIntakeMotorId);

        // Config
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(Constants.Conveyor.kIntakeInverted);
		
		// Follow
		shooter2.follow(shooter1);
		shooter3.follow(shooter1);
		shooter1.setInverted(Constants.Shooter.IS_SHOOTER_INVERTED);
		shooter2.setInverted(InvertType.OpposeMaster);
		shooter3.setInverted(InvertType.OpposeMaster);
		shooter1.setNeutralMode(NeutralMode.Coast);
		shooter2.setNeutralMode(NeutralMode.Coast);
        shooter3.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
                Constants.Shooter.kCurrentLimit, Constants.Shooter.kCurrentLimit, 1);
        shooter1.configSupplyCurrentLimit(currentConfig);

        shooter1.configClosedloopRamp(Constants.Shooter.kRampRate); // Ramp rate for Velocity PID
        shooter1.configOpenloopRamp(Constants.Shooter.kRampRate); // Ramp rate for open loop control

        // Follow
        shooter2.follow(shooter1);
        shooter3.follow(shooter1);
        configPIDF(Constants.Shooter.kP, 0, 0, Constants.Shooter.kF, Constants.Shooter.kPIDIdx);
    }

    public void setIntakeMotors(double intakeMotorSpeed, double shooterMotorSpeed) {
        intakeMotor.set(ControlMode.PercentOutput, intakeMotorSpeed);
        shooter1.set(ControlMode.PercentOutput, shooterMotorSpeed);
    }

    public void stopShooter() {
        shooter1.set(ControlMode.PercentOutput, 0);
    }

    public void setPercent(double percentOutput) {
        shooter1.set(TalonFXControlMode.PercentOutput, percentOutput);
    }

    public void setVelocity(double nativeUnits) {
        shooter1.set(TalonFXControlMode.Velocity, nativeUnits);
    }

    public void configPIDF(double p, double i, double d, double f, int idx) {
        shooter1.config_kP(idx, p);
        shooter1.config_kI(idx, i);
        shooter1.config_kD(idx, d);
        shooter1.config_kF(idx, f);
    }

    public double getVelocity() {
        return shooter1.getSelectedSensorVelocity();
    }

    public double getVoltage() {
        return shooter1.getMotorOutputVoltage();
    }

    public double getCurrent() {
        return shooter1.getStatorCurrent();
    }

    public int interpolateY(double currentX, int[] yValues) {
		double[] xValues = distanceTable;
        int startIndex = 0;
        int lowY;
        int highY;
        while (currentX > xValues[startIndex]) {
            startIndex++;
        }
        int endIndex = startIndex + 1;
        // Find the velocities directly surrounding the distance of the robot
        if (endIndex < xValues.length) {
            lowY = yValues[startIndex];
            highY = yValues[endIndex];
        } else {
            // TODO: This probably shouldn't throw an exception and crash the code
            throw new IndexOutOfBoundsException("Distance too far");
        }
        // Start with base velocity, then add the weighted average of the low and high
        // velocities to calculate the interpolated velocity. See
        // https://en.wikipedia.org/wiki/Linear_interpolation for more details
        int interpolatedY = (int) (lowY
                + (((highY - lowY) / (xValues[endIndex] - xValues[startIndex])) * (currentX - xValues[startIndex])));
        return interpolatedY;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
