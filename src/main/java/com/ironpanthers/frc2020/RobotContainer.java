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
import com.ironpanthers.frc2020.commands.vision.TurnToTarget;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
	private final Shooter shooter = new Shooter();
	private final ConveyorBelt conveyorBelt = new ConveyorBelt();
	private final Arm arm = new Arm();

	private static final Joystick joystick = new Joystick(Constants.OI.kDriverAJoystickPort);
	private static final Joystick armJoystick = new Joystick(Constants.OI.kDriverBJoystickPort);

	// Driver A Buttons
	private final JoystickButton intakeButton = new JoystickButton(joystick, Constants.OI.kIntakeButton);
	private final JoystickButton shooterButton = new JoystickButton(joystick, 5);
	private final JoystickButton turnToTargetButton = new JoystickButton(joystick, 6);

	// Driver B Buttons
	private final JoystickButton manualArm = new JoystickButton(armJoystick, Constants.OI.kManualArmButton);
	private final JoystickButton driverBIntake = new JoystickButton(armJoystick, Constants.OI.kDriverBIntakeButton);
	private final JoystickButton zeroArm = new JoystickButton(armJoystick, Constants.OI.kZeroArmButton);
	private final JoystickButton farShotPosition = new JoystickButton(armJoystick, Constants.OI.kFarShotButton);
	private final JoystickButton framePerimeterHeightPosition = new JoystickButton(armJoystick,Constants.OI.kFramePerimeterHeightButton);

	public RobotContainer() {
		drive.setDefaultCommand(
				new ManualDriveCommand(joystick::getY, joystick::getX, new JoystickButton(joystick, 1), drive));

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
		turnToTargetButton.whenPressed(new TurnToTarget(drive));
		// Driver B
		manualArm.whileHeld(new ManualArmCommand(arm, armJoystick::getY));
		driverBIntake.whileHeld(new IntakeSequence(shooter, conveyorBelt, driverBIntake::get));
		zeroArm.whenPressed(new ZeroArm(arm));
		farShotPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.kFarShotHeightNativeUnits));
		framePerimeterHeightPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.kFrameConstrainedHeightNativeUnits));
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
