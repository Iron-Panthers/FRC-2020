/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorOuttake extends CommandBase {

	private ConveyorBelt conveyor;
	private Timer timer;
	private double speed;
	private Arm arm;
	private boolean isArm;
	/**
	 * Creates a new ConveyorOuttake.
	 */
	public ConveyorOuttake(ConveyorBelt conveyor) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(conveyor);
		this.conveyor = conveyor;
		this.timer = new Timer();
		speed = Constants.Conveyor.kConveyorSpeedClose;
		isArm = false;
	}

	public ConveyorOuttake(ConveyorBelt conveyor, double conveyorSpeed, Arm arm) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.conveyor = conveyor;
		this.timer = new Timer();
		this.arm = arm;
		isArm = true;
		speed = conveyorSpeed;
		addRequirements(conveyor, arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (isArm) {
			arm.engageBrake();
		}
		timer.reset();
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		conveyor.setPower(speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		conveyor.stop();
		if (isArm) {
			arm.releaseBrake();
		}
		conveyor.ballsHeld = 0;
		conveyor.lastBallRan = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (speed > 0.5) {
			return timer.hasElapsed(Constants.Conveyor.kConveyorTime * 4/3);
		}
		else {
			return timer.hasElapsed(Constants.Conveyor.kConveyorTime * 2 * 4/3);
		}
		
	}
}
