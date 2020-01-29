/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

import com.ironpanthers.frc2020.auto.commands.TestAutonomous;
import com.ironpanthers.frc2020.commands.IntakeSequence;
import com.ironpanthers.frc2020.commands.ManualDriveCommand;
import com.ironpanthers.frc2020.commands.ResetConveyor;
import com.ironpanthers.frc2020.commands.ShootAtVelocity;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Drive drive = new Drive();

	public static final Joystick joystick = new Joystick(Constants.OI.JOYSTICK_PORT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public final Shooter shooter = new Shooter();
	public final ConveyorBelt conveyorBelt = new ConveyorBelt();
	public final JoystickButton intakeButton = new JoystickButton(joystick, Constants.OI.INTAKE_BUTTON_PORT);
	public final JoystickButton plzWorkButton = new JoystickButton(joystick, Constants.OI.SHOOT_WITH_VELOCITY_PORT);

	public RobotContainer() {
		drive.setDefaultCommand(
				new ManualDriveCommand(joystick::getY, joystick::getX, new JoystickButton(joystick, 1), drive));

		SmartDashboard.putData("delete soon. fow now: reset drive", new RunCommand(() -> drive.reset()));

		// Configure the button bindings

		configureButtonBindings();
	}

	/**
	 * Applies all button->command mappings.
	 */
	private void configureButtonBindings() {
		intakeButton.whileHeld(new IntakeSequence(shooter, conveyorBelt, intakeButton::get));
		intakeButton.whenReleased(new ResetConveyor(conveyorBelt));
		plzWorkButton.whileHeld(new ShootAtVelocity(shooter, conveyorBelt, Constants.Shooter.SHOOTER_TEST_VELOCITY));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return new TestAutonomous(drive);
	}
}
