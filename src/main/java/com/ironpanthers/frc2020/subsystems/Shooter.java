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
    private final TalonFX shooter1 = new TalonFX(Constants.Shooter.kShooter1Id);
    private final TalonFX shooter2 = new TalonFX(Constants.Shooter.kShooter2Id);
    private final TalonFX shooter3 = new TalonFX(Constants.Shooter.kShooter2Id);
    private final TalonFX intakeMotor = new TalonFX(Constants.Shooter.kShooter2Id);;

    // TODO clean up access to these values
    public int velocity;
    public boolean fullShotDone = false;

    // TODO these values must be tuned
    private final double[] distanceTable = { 0, 120.0, 240.0, 408.0 }; // Inches
    public final double[] velocityTable = { Constants.Shooter.kCloseVelocity, Constants.Shooter.kInitiationVelocity,
            Constants.Shooter.kFarVelocity }; // Units/100ms
    public final double[] armPosTable = { Constants.Arm.kCloseShotDegrees, Constants.Arm.kInitiationLineDegrees,
            Constants.Arm.kFarShotDegrees };

    /**
     * Create a new Shooter subsystem. As usual, only one of these should ever be
     * constructed.
     */
    public Shooter() {
        shooter2.follow(shooter1);
        shooter3.follow(shooter1);
        shooter1.setInverted(Constants.Shooter.IS_SHOOTER_INVERTED);
        shooter2.setInverted(InvertType.FollowMaster);
        shooter3.setInverted(InvertType.FollowMaster);
        shooter1.setNeutralMode(NeutralMode.Coast);
        shooter2.setNeutralMode(NeutralMode.Coast);
        shooter3.setNeutralMode(NeutralMode.Coast);

        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(Constants.Conveyor.kIntakeInverted);

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
                Constants.Shooter.kCurrentLimit, Constants.Shooter.kCurrentLimit, 1);
        shooter1.configSupplyCurrentLimit(currentConfig);
        shooter1.configClosedloopRamp(Constants.Shooter.kRampRate); // Ramp rate for Velocity PID
        shooter1.configOpenloopRamp(Constants.Shooter.kRampRate); // Ramp rate for open loop control
        shooter2.follow(shooter1);
        shooter3.follow(shooter1);

        configPIDF(Constants.Shooter.kP, 0, 0, Constants.Shooter.kF, Constants.Shooter.kPIDIdx);
    }

    /**
     * Sets a pair of percent outputs for the intake and flywheel (respectively).
     * 
     * @param intakeMotorSpeed  The target output, which should be between -1.0 and
     *                          1.0, with 0.0 indicating the intake being stopped.
     * @param shooterMotorSpeed The target output, which should be between -1.0 and
     *                          1.0, with 0.0 indicating the flywheel being stopped.
     */
    public void setIntakeMotors(double intakeMotorSpeed, double shooterMotorSpeed) {
        intakeMotor.set(ControlMode.PercentOutput, intakeMotorSpeed);
        shooter1.set(ControlMode.PercentOutput, shooterMotorSpeed);
    }

    /**
     * Equivalent to calling {@link #setPercent(double)} with a value of 0.
     */
    public void stopShooter() {
        shooter1.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Set a percent output for the flywheel.
     * 
     * @param percentOutput The target output, which should be between -1.0 and 1.0,
     *                      with 0.0 indicating the flywheel being stopped.
     */
    public void setPercent(double percentOutput) {
        shooter1.set(TalonFXControlMode.PercentOutput, percentOutput);
    }

    /**
     * Set a velocity setpoint for the flywheel.
     * 
     * @param nativeUnits The target velocity for the motor, in native units of
     *                    position change / 100ms.
     */
    public void setVelocity(double nativeUnits) {
        shooter1.set(TalonFXControlMode.Velocity, nativeUnits);
    }

    /**
     * Do an in-place PIDF reconfiguration for the flywheel master motor.
     * <p>
     * For a more thorough explanation of how PID works, look elsewhere.
     */
    public void configPIDF(double p, double i, double d, double f, int idx) {
        shooter1.config_kP(idx, p);
        shooter1.config_kI(idx, i);
        shooter1.config_kD(idx, d);
        shooter1.config_kF(idx, f);
    }

    /**
     * Returns the actual velocity of the motor.
     * 
     * @return Velocity in native units / 100ms.
     */
    public double getVelocity() {
        return shooter1.getSelectedSensorVelocity();
    }

    /**
     * Returns the output voltage.
     * 
     * @return The applied voltage of the motor (in volts).
     */
    public double getVoltage() {
        return shooter1.getMotorOutputVoltage();
    }

    /**
     * Returns the output current.
     * 
     * @return The stator current of the motor (in amps).
     */
    public double getCurrent() {
        return shooter1.getStatorCurrent();
    }

    // TODO move this out of here
    // TODO figure out what should be provided as parameters, what should use
    // members, etc.
    public int interpolateY(double currentX, double[] yValues) {
        double[] xValues = distanceTable;
        int endIndex = 0;
        double lowY;
        double highY;
        while (currentX > xValues[endIndex]) {
            endIndex++;
        }
        int startIndex = endIndex - 1;
        // Find the velocities directly surrounding the distance of the robot
        if (endIndex < xValues.length - 1) {
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
