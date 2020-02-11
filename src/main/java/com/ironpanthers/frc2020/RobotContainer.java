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
import com.ironpanthers.frc2020.commands.intake.Outtake;
import com.ironpanthers.frc2020.commands.intake.ResetConveyor;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence;
import com.ironpanthers.frc2020.commands.shooter.StopShooter;
import com.ironpanthers.frc2020.commands.vision.TurnToTarget;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;
import com.ironpanthers.util.AutoSelector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private final AutoSelector autoSelector = new AutoSelector();
	private final SteeringAdjuster steerer = new SteeringAdjuster(arm::getHorizontalDistance, arm::getDiagonalDistance);

	private static final Joystick joystickA = new Joystick(Constants.OI.kDriverAJoystickPort);
	private static final Joystick joystickB = new Joystick(Constants.OI.kDriverBJoystickPort);

	// Driver A Buttons
	private final JoystickButton driverAStopShooterButton = new JoystickButton(joystickA, 3);
	private final JoystickButton intakeButton = new JoystickButton(joystickA, Constants.OI.kIntakeButton);
	private final JoystickButton shooterButton = new JoystickButton(joystickA, 5);
	private final JoystickButton turnToTargetButton = new JoystickButton(joystickA, 6);


	// Driver B Buttons
	private final JoystickButton manualArm = new JoystickButton(joystickB, Constants.OI.kManualArmButton);
	private final JoystickButton driverBIntake = new JoystickButton(joystickB, Constants.OI.kDriverBIntakeButton);
	private final JoystickButton zeroArm = new JoystickButton(joystickB, Constants.OI.kZeroArmButton);
	private final JoystickButton farShotPosition = new JoystickButton(joystickB, Constants.OI.kFarShotButton);
	private final JoystickButton framePerimeterHeightPosition = new JoystickButton(joystickB,Constants.OI.kFramePerimeterHeightButton);
	private final JoystickButton emergencyOuttake = new JoystickButton(joystickB, Constants.OI.kEmergencyOuttakeButton);

	public RobotContainer() {
		drive.setDefaultCommand(
				new ManualDriveCommand(joystickA::getY, joystickA::getX, new JoystickButton(joystickA, 1), drive));

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
		driverAStopShooterButton.whenPressed(new StopShooter(shooter));

		turnToTargetButton.whenPressed(new TurnToTarget(drive, steerer, LimelightWrapper.getLimelightWrapperFront()::targetVisible));
		// Driver B
		manualArm.whileHeld(new ManualArmCommand(arm, joystickB::getY));
		driverBIntake.whileHeld(new IntakeSequence(shooter, conveyorBelt, driverBIntake::get));
		zeroArm.whenPressed(new ZeroArm(arm));
		farShotPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.kFarShotHeightNativeUnits));
		framePerimeterHeightPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.kFrameConstrainedHeightNativeUnits));
		emergencyOuttake.whileHeld(new Outtake(shooter));
	}

	public void smartDashboard() {
		SmartDashboard.putNumber("Auto Selector Value", autoSelector.getAutoPotNumber());
		SmartDashboard.putNumber("Auto Selector Voltage", autoSelector.getPotVoltage());
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
