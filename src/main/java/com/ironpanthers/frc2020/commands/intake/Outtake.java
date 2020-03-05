package com.ironpanthers.frc2020.commands.intake;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Outtake extends CommandBase {
	/**
	 * Creates a new Intake.
	 */
	ConveyorBelt conveyor;
	Shooter shooter;
	int counter;
	Timer timer;
	public Outtake(Shooter shooter) {
		counter = 0;
		this.shooter = shooter;
		timer = new Timer();

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		shooter.setIntakeMotors(-Constants.Conveyor.kIntakeRollerSpeed, -Constants.Conveyor.kIntakeFlywheelSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.setIntakeMotors(0, 0);	
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.hasElapsed(1);
	}
}
