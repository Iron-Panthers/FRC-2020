// THIS CLASS KEPT ONLY FOR TESTING PURPOSES

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.arm;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuneArmPositionPID extends CommandBase {
	private Arm arm;
	private double p, i, d, f;
	private double pos;

	/**
	 * Creates a new TuneArmPositionPID.
	 */
	public TuneArmPositionPID(Arm arm) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.arm = arm;
		addRequirements(arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putNumber("Arm P", Constants.Arm.kP);
		SmartDashboard.putNumber("Arm I", Constants.Arm.kI);
		SmartDashboard.putNumber("Arm D", Constants.Arm.kD);
		SmartDashboard.putNumber("Arm F", Constants.Arm.kF);
		SmartDashboard.putNumber("Target Position", 0);
		p = SmartDashboard.getNumber("Arm P", Constants.Arm.kP);
		i = SmartDashboard.getNumber("Arm I", Constants.Arm.kI);
		d = SmartDashboard.getNumber("Arm D", Constants.Arm.kD);
		f = SmartDashboard.getNumber("Arm F", Constants.Arm.kF);
		pos = SmartDashboard.getNumber("Target Angle", 0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double nP = SmartDashboard.getNumber("Arm P", Constants.Arm.kP);
		double nI = SmartDashboard.getNumber("Arm I", Constants.Arm.kI);
		double nD = SmartDashboard.getNumber("Arm D", Constants.Arm.kD);
		double nF = SmartDashboard.getNumber("Arm F", Constants.Arm.kF);
		double nPos = SmartDashboard.getNumber("Target Position", 0);
		boolean update = false;
		// Only reconfigure PIDF if it gets changed
		if (nP != p) {
			p = nP;
			update = true;
		}
		if (nI != i) {
			i = nI;
			update = true;
		}
		if (nD != d) {
			d = nD;
			update = true;
		}
		if (nF != f) {
			f = nF;
			update = true;
		}
		if (update) {
			arm.configPIDF(p, i, d, f, Constants.Arm.kPIDIdx);
		}
		if (nPos != pos) {
			pos = nPos;
		}
		arm.setPosition(pos);
		SmartDashboard.putNumber("Arm Voltage", arm.getOutputVoltage());
		SmartDashboard.putNumber("Arm Current", arm.getOutputCurrent());

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
