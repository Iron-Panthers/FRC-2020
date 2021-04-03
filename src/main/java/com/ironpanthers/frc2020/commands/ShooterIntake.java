package com.ironpanthers.frc2020.commands;

import java.util.function.BooleanSupplier;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterIntake extends CommandBase {
	Shooter shooter;
	ConveyorBelt conveyor;
	BooleanSupplier button;

	/**
	 * Creates a new ShooterIntake.
	 */
	public ShooterIntake(Shooter shooter, ConveyorBelt conveyor, BooleanSupplier button) {
		this.button = button;
		this.conveyor = conveyor;
		this.shooter = shooter;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter, conveyor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (conveyor.conveyorFull())
			cancel();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (conveyor.conveyorFull())
			cancel();
        shooter.setVelocity(-13_000);
        shooter.setIntakeMotor(-Constants.Conveyor.kIntakeRollerSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.stopShooter();
		shooter.setIntakeMotor(0);
		if (!interrupted && conveyor.ballsHeld < 3) {
			conveyor.ballsHeld++;
		} 
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return conveyor.getBannerSensor();
	}
}
