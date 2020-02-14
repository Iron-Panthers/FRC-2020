/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import java.util.function.DoubleSupplier;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArmCommand extends CommandBase {
	private Arm arm;
	private DoubleSupplier forward;
	/**
	 * Creates a new ManualArmCommand.
	 */
	public ManualArmCommand(Arm arm, DoubleSupplier forward) {
		addRequirements(arm);
		this.arm = arm;
		this.forward = forward;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		arm.stop();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		arm.setPower(forward.getAsDouble() * Constants.Arm.kMaxManualSpeed);
		SmartDashboard.putNumber("arm/angle", arm.getAngle());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.setVoltage(arm.getFeedForward());
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
