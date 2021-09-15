/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ironpanthers.frc2020;

import java.io.IOException;

import com.ironpanthers.frc2020.auto.commands.Shoot3Baseline;
import com.ironpanthers.frc2020.auto.commands.TestAutonomous;
import com.ironpanthers.frc2020.commands.FullShooterSequence;
import com.ironpanthers.frc2020.commands.arm.ArmHold;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.arm.ManualArmCommand;
import com.ironpanthers.frc2020.commands.climb.ClimbDown;
import com.ironpanthers.frc2020.commands.climb.ClimbUp;
import com.ironpanthers.frc2020.commands.drive.GearShift;
import com.ironpanthers.frc2020.commands.drive.ManualDriveCommand;
import com.ironpanthers.frc2020.commands.intake.IntakeSequence;
import com.ironpanthers.frc2020.commands.intake.Outtake;
import com.ironpanthers.frc2020.commands.shooter.SetShooterVelocityEmergency;
import com.ironpanthers.frc2020.commands.shooter.Shoot;
import com.ironpanthers.frc2020.commands.shooter.StopShooter;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.Climb;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LightMode;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.frc2020.util.SteeringAdjuster;
import com.ironpanthers.util.AutoSelector;

import edu.wpi.first.wpilibj.Compressor;
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
	private final SteeringAdjuster steerer = new SteeringAdjuster(limelightWrapper, arm::getHorizontalDistance, arm);
	private final Climb climb = new Climb();

	// Hardware
	private final AutoSelector autoSelector = new AutoSelector();
	private final Compressor compressor = new Compressor();

	// Joysticks
	private final Joystick joystickA = new Joystick(Constants.OI.kDriverAJoystickPort);
	private final Joystick joystickB = new Joystick(Constants.OI.kDriverBJoystickPort);

	// Hubert
	// Driver A Buttons
	private final JoystickButton driveShift = new JoystickButton(joystickA, Constants.OI.kDriveShiftButton); // 2
	private final JoystickButton intakeButton = new JoystickButton(joystickA, Constants.OI.kIntakeButton); // 4
	private final JoystickButton climbDown = new JoystickButton(joystickA, Constants.OI.kClimbDown); // 5
	private final JoystickButton climbUp = new JoystickButton(joystickA, Constants.OI.kClimbUp); // 6
	private final JoystickButton driverAStopShooterButton = new JoystickButton(joystickA,
			Constants.OI.kStopShooterButton); // 7
	private final JoystickButton emergencyShootA = new JoystickButton(joystickA, Constants.OI.kEmergencyShootButton); // 8
	private final JoystickButton armToClimb = new JoystickButton(joystickA, Constants.OI.kArmToClimb); // 9

	// Cherilyn
	private final JoystickButton manualArm = new JoystickButton(joystickB, Constants.OI.kManualArmButton); // 1
	private final JoystickButton driverBIntake = new JoystickButton(joystickB, Constants.OI.kDriverBIntakeButton); // 2
	private final JoystickButton emergencyOuttake = new JoystickButton(joystickB, Constants.OI.kEmergencyOuttakeButton); // 3
	private final JoystickButton stopShooterB = new JoystickButton(joystickB, Constants.OI.kStopShooterButtonB); // 4
	private final JoystickButton closeShot = new JoystickButton(joystickB, Constants.OI.kCloseShotButton); // 12
	private final JoystickButton lineShot = new JoystickButton(joystickB, Constants.OI.kShootInitiation); // 10
	private final JoystickButton closeTrench = new JoystickButton(joystickB, Constants.OI.kCloseTrenchButton); // 11
	private final JoystickButton controlPanel = new JoystickButton(joystickB, Constants.OI.kControlPanel); // 9
	private final JoystickButton emergencyShootB = new JoystickButton(joystickB, Constants.OI.kEmergencyShootButton); // 8
	private final JoystickButton shootClose = new JoystickButton(joystickB, Constants.OI.kShootClose); // 6
	private final JoystickButton shootFar = new JoystickButton(joystickB, Constants.OI.kShootFar); // 7

	public RobotContainer() {
		drive.setDefaultCommand(
				new ManualDriveCommand(joystickA::getY, joystickA::getX, new JoystickButton(joystickA, 1), drive));
		arm.setDefaultCommand(new ArmHold(arm));
		compressor.setClosedLoopControl(true);
		// Configure the button bindings
		configureButtonBindings();
	}

	public void initializeAuto() {
		arm.calibrateCANCoder();
		resetBallsHeld();
	}

	public void initializeTeleop() {
		arm.calibrateCANCoder();
	}

	public void setLightMode(LightMode mode) {
		limelightWrapper.setLightMode(mode);
	}

	/**
	 * Applies all button->command mappings.
	 */
	private void configureButtonBindings() {
		// Driver A
		driveShift.whileHeld(new GearShift(drive));
		intakeButton.whileHeld(new IntakeSequence(shooter, conveyorBelt, intakeButton::get));
		intakeButton.whenReleased(new Outtake(shooter, conveyorBelt));
		driverAStopShooterButton.whenPressed(new StopShooter(shooter));
		climbDown.whileHeld(new ClimbDown(climb));
		climbUp.whileHeld(new ClimbUp(climb));
		armToClimb.whenPressed(new ArmToTarget(arm, Constants.Arm.kClimbDegrees));

		// Driver B
		stopShooterB.whenPressed(new StopShooter(shooter));
		shootClose.whenPressed(new Shoot(drive, steerer, conveyorBelt, arm, limelightWrapper, shooter,
				Constants.Conveyor.kConveyorSpeedClose));
		shootFar.whenPressed(new Shoot(drive, steerer, conveyorBelt, arm, limelightWrapper, shooter,
				Constants.Conveyor.kConveyorSpeedFar));
		manualArm.whileHeld(new ManualArmCommand(arm, joystickB::getY));
		driverBIntake.whileHeld(new IntakeSequence(shooter, conveyorBelt, driverBIntake::get));
		driverBIntake.whenReleased(new Outtake(shooter, conveyorBelt));
		emergencyOuttake.whileHeld(new Outtake(shooter, conveyorBelt));
		closeShot.whenPressed(new FullShooterSequence(steerer, drive, arm, Constants.Arm.kCloseShotDegrees, shooter,
				Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper,
				Constants.Shooter.kCloseVelocity));
		lineShot.whenPressed(new FullShooterSequence(steerer, drive, arm, Constants.Arm.kInitiationLineDegrees, shooter,
				Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper,
				Constants.Shooter.kInitiationVelocity));
		closeTrench.whenPressed(new FullShooterSequence(steerer, drive, arm, Constants.Arm.kCloseTrenchDegrees, shooter,
				Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper,
				Constants.Shooter.kCloseTrenchVelocity));
		controlPanel.whenPressed(new FullShooterSequence(steerer, drive, arm, Constants.Arm.kFarShotDegrees, shooter,
				Constants.Shooter.kInnerGoalThreshold, conveyorBelt, limelightWrapper, Constants.Shooter.kFarVelocity));
		emergencyShootA.whileHeld(new SetShooterVelocityEmergency(shooter, Constants.Shooter.kCloseVelocity,
				Constants.Shooter.kOuterGoalThreshold, conveyorBelt, limelightWrapper));
		emergencyShootB.whileHeld(new SetShooterVelocityEmergency(shooter, Constants.Shooter.kCloseVelocity,
				Constants.Shooter.kOuterGoalThreshold, conveyorBelt, limelightWrapper));

	}

	public void resetBallsHeld() {
		conveyorBelt.ballsHeld = 3;
	}

	public void smartDashboard() {
		SmartDashboard.putNumber("balls held", conveyorBelt.ballsHeld);
		SmartDashboard.putNumber("Auto Selector Value", autoSelector.getAutoPotNumber());
		SmartDashboard.putNumber("Auto Selector Voltage", autoSelector.getPotVoltage());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() throws IOException {
		try {
			return new Shoot3Baseline(arm, Constants.Arm.kInitiationLineDegrees, shooter,
					Constants.Shooter.kInitiationVelocity, Constants.Shooter.kInnerGoalThreshold, conveyorBelt, drive,
					limelightWrapper).andThen(new TestAutonomous(drive));
		} catch (IOException e) {
			throw e;
		}
	}
}
