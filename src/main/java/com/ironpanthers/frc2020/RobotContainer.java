/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

import com.ironpanthers.frc2020.auto.commands.Shoot3Baseline;
import com.ironpanthers.frc2020.auto.commands.TestAutonomous;
import com.ironpanthers.frc2020.commands.FullShooterSequence;
import com.ironpanthers.frc2020.commands.arm.ArmAndSpinShooter;
import com.ironpanthers.frc2020.commands.arm.ArmHold;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.arm.ManualArmCommand;
import com.ironpanthers.frc2020.commands.arm.ZeroArm;
import com.ironpanthers.frc2020.commands.drive.ManualDriveCommand;
import com.ironpanthers.frc2020.commands.intake.EmergencyIntake;
import com.ironpanthers.frc2020.commands.intake.IntakeSequence;
import com.ironpanthers.frc2020.commands.intake.Outtake;
import com.ironpanthers.frc2020.commands.intake.ResetConveyor;
import com.ironpanthers.frc2020.commands.shooter.ShootQuickly;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence;
import com.ironpanthers.frc2020.commands.shooter.StopShooter;
import com.ironpanthers.frc2020.commands.vision.TurnToTarget;
import com.ironpanthers.frc2020.commands.vision.VisionTesting;
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
	private final LimelightWrapper limelightWrapper = new LimelightWrapper();
	private final Drive drive = new Drive();
	private final Shooter shooter = new Shooter();
	private final ConveyorBelt conveyorBelt = new ConveyorBelt();
	private final Arm arm = new Arm(limelightWrapper);
	private final SteeringAdjuster steerer = new SteeringAdjuster(limelightWrapper);
	
	private final AutoSelector autoSelector = new AutoSelector();

	private static final Joystick joystickA = new Joystick(Constants.OI.kDriverAJoystickPort);
	private static final Joystick joystickB = new Joystick(Constants.OI.kDriverBJoystickPort);

	// Driver A Buttons
	private final JoystickButton driverAStopShooterButton = new JoystickButton(joystickA,
			Constants.OI.kStopShooterButton); // 3
	private final JoystickButton intakeButton = new JoystickButton(joystickA, Constants.OI.kIntakeButton); // 4
	private final JoystickButton turnToTargetButton = new JoystickButton(joystickA, Constants.OI.kAutoAlign); // 6
	private final JoystickButton shootFar = new JoystickButton(joystickA, Constants.OI.kShootFar); // 8
	private final JoystickButton shootInitiation = new JoystickButton(joystickA, Constants.OI.kShootInitiation); // 10
																													// Make
																													// new
																													// constant
																													// for
																													// this
																													// at
																													// 11
	private final JoystickButton shootClose = new JoystickButton(joystickA, Constants.OI.kShootClose); // 12

	// Driver B Buttons
	private final JoystickButton manualArm = new JoystickButton(joystickB, Constants.OI.kManualArmButton);
	private final JoystickButton driverBIntake = new JoystickButton(joystickB, Constants.OI.kDriverBIntakeButton);
	private final JoystickButton zeroArm = new JoystickButton(joystickB, Constants.OI.kZeroArmButton);
	private final JoystickButton farShotPosition = new JoystickButton(joystickB, Constants.OI.kFarShotButton);
	private final JoystickButton framePerimeterHeightPosition = new JoystickButton(joystickB,
			Constants.OI.kFramePerimeterHeightButton);
	private final JoystickButton closeShotPosition = new JoystickButton(joystickB, Constants.OI.kCloseShotButton);
	private final JoystickButton emergencyOuttake = new JoystickButton(joystickB, Constants.OI.kEmergencyOuttakeButton);
	private final JoystickButton emergencyIntake = new JoystickButton(joystickB, Constants.OI.kEmergencyIntakeButton);
	private final JoystickButton autoShotHeight = new JoystickButton(joystickB, Constants.OI.kAutoShotHeightButton);
	// private final JoystickButton getDistance = new JoystickButton(joystickB, Constants.OI.kLimelightTest);
	private final JoystickButton fullShooterSequence = new JoystickButton(joystickB, 4);

	public RobotContainer() {
		drive.setDefaultCommand(
				new ManualDriveCommand(joystickA::getY, joystickA::getX, new JoystickButton(joystickA, 1), drive));
		arm.setDefaultCommand(new ArmHold(arm));
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
		shootFar.whileHeld(new ShooterSequence(shooter, conveyorBelt, Constants.Shooter.kFarVelocity,
				Constants.Shooter.kInnerGoalThreshold, limelightWrapper));
		driverAStopShooterButton.whenPressed(new StopShooter(shooter));
		shootClose.whileHeld(new ShootQuickly(shooter, conveyorBelt, Constants.Shooter.kFarVelocity,
				Constants.Shooter.kOuterGoalThreshold,limelightWrapper));
		shootInitiation.whileHeld(new ShooterSequence(shooter, conveyorBelt, Constants.Shooter.kInitiationVelocity, Constants.Shooter.kInnerGoalThreshold,limelightWrapper));
		turnToTargetButton.whenPressed(new TurnToTarget(drive, steerer,limelightWrapper));
		// Driver B
		zeroArm.whenPressed(new ZeroArm(arm));
		manualArm.whileHeld(new ManualArmCommand(arm, joystickB::getY));
		driverBIntake.whileHeld(new IntakeSequence(shooter, conveyorBelt, driverBIntake::get));
		closeShotPosition.whenPressed(new ArmAndSpinShooter(arm, Constants.Arm.kCloseShotHeightNativeUnits, shooter, Constants.Shooter.kCloseVelocity, Constants.Shooter.kOuterGoalThreshold, conveyorBelt, limelightWrapper));
		farShotPosition.whenPressed(new ArmAndSpinShooter(arm, Constants.Arm.kFarShotHeightNativeUnits, shooter, Constants.Shooter.kFarVelocity, Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper));
		framePerimeterHeightPosition.whenPressed(new ArmToTarget(arm, Constants.Arm.kFrameConstrainedHeightNativeUnits, limelightWrapper));
		emergencyOuttake.whileHeld(new Outtake(shooter));
		emergencyIntake.whileHeld(new EmergencyIntake(shooter, conveyorBelt, emergencyIntake::get));
		autoShotHeight.whenPressed(new ArmAndSpinShooter(arm, Constants.Arm.kInitiationLineHeight, shooter, Constants.Shooter.kInitiationVelocity, Constants.Shooter.kOuterGoalThreshold, conveyorBelt, limelightWrapper));
		fullShooterSequence.whenPressed(new FullShooterSequence(steerer, drive, arm, Constants.Arm.kInitiationLineHeight, shooter, Constants.Shooter.kInitiationVelocity, Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper));
		// getDistance.whileHeld(new VisionTesting(limelightWrapper, arm));
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
		return new Shoot3Baseline(arm, Constants.Arm.kInitiationLineHeight, shooter, Constants.Shooter.kInitiationVelocity, Constants.Shooter.kInnerGoalThreshold, conveyorBelt, drive, limelightWrapper);
	}
}
