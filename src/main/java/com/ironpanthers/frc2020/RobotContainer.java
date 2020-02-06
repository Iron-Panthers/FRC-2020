/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

import com.ironpanthers.frc2020.auto.commands.TestAutonomous;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.arm.ManualArmCommand;
import com.ironpanthers.frc2020.commands.arm.ZeroArm;
import com.ironpanthers.frc2020.commands.drive.ManualDriveCommand;
import com.ironpanthers.frc2020.commands.intake.IntakeSequence;
import com.ironpanthers.frc2020.commands.intake.ResetConveyor;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence;
import com.ironpanthers.frc2020.subsystems.Arm;
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

	public static final Joystick joystick = new Joystick(Constants.OI.DRIVE_JOYSTICK);
	public static final Joystick armJoystick = new Joystick(Constants.OI.ARM_JOYSTICK);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public final Shooter shooter = new Shooter();
	public final ConveyorBelt conveyorBelt = new ConveyorBelt();
	public final Arm arm = new Arm();
	// Driver A Buttons
	public final JoystickButton intakeButton = new JoystickButton(joystick, Constants.OI.INTAKE_BUTTON_PORT);
	public final JoystickButton shooterButton = new JoystickButton(joystick, 5);
	public final JoystickButton turnToTargetButton = new JoystickButton(joystick, 6);
	// Driver B Buttons
	public final JoystickButton manualArm = new JoystickButton(armJoystick, Constants.OI.MANUAL_ARM_BUTTON);
	public final JoystickButton driverBIntake = new JoystickButton(armJoystick, Constants.OI.DRIVER_B_INTAKE_BUTTON);
	public final JoystickButton zeroArm = new JoystickButton(armJoystick, Constants.OI.ZERO_ARM_BUTTON);
	public final JoystickButton farShotPosition = new JoystickButton(armJoystick, Constants.OI.FAR_SHOT_POSITION_BUTTON);
	public final JoystickButton framePerimeterHeightPosition = new JoystickButton(armJoystick, Constants.OI.FRAME_PERIMETER_HEIGHT_BUTTON);

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
		// Driver A
		intakeButton.whileHeld(new IntakeSequence(shooter, conveyorBelt, intakeButton::get));
		intakeButton.whenReleased(new ResetConveyor(conveyorBelt));
		shooterButton.whileHeld(new ShooterSequence(shooter, conveyorBelt));
		// Driver B
		manualArm.whileHeld(new ManualArmCommand(arm, armJoystick::getY));
		driverBIntake.whileHeld(new IntakeSequence(shooter, conveyorBelt, driverBIntake::get));
		zeroArm.whenPressed(new ZeroArm(arm));
		farShotPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.ARM_FAR_SHOT));
		framePerimeterHeightPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.ARM_FRAME_PERIMETER_HEIGHT));
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
