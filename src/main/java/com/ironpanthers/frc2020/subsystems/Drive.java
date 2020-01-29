package com.ironpanthers.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.sensors.PigeonIMU;
import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.util.PhoenixUtil;
import com.ironpanthers.util.PoseLoggingTable;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the robot's drivetrain.
 */
public class Drive extends SubsystemBase {
    private final TalonFX left1 = new TalonFX(Constants.Drive.kLeft1Id);
    private final TalonFX left2 = new TalonFX(Constants.Drive.kLeft2Id);
    private final TalonFX right1 = new TalonFX(Constants.Drive.kRight1Id);
	private final TalonFX right2 = new TalonFX(Constants.Drive.kRight2Id);
	
    // private final PigeonIMU gyro = new PigeonIMU(new TalonSRX(Constants.Drive.kPigeonTalonId));

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.Drive.kTrackWidthMeters);
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(heading());

    private final SimpleMotorFeedforward characterizedFeedforward = new SimpleMotorFeedforward(Constants.Drive.kS,
            Constants.Drive.kV, Constants.Drive.kA);

    private final PIDController left = new PIDController(Constants.Drive.kP, 0.0, 0.0);
    private final PIDController right = new PIDController(Constants.Drive.kP, 0.0, 0.0);

    private Pose2d currentPose = new Pose2d();

    /**
     * Create a new Drive subsystem.
     */
    public Drive() {
        left1.configFactoryDefault();
        left2.configFactoryDefault();
        right1.configFactoryDefault();
        right2.configFactoryDefault();

        // Followers
        left2.follow(left1);
        right2.follow(right1);

        // Configuration
        left1.setInverted(true);
        left2.setInverted(true);
        right1.setInverted(false);
		right2.setInverted(false);
		
		SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true,
				Constants.Drive.DRIVE_CURRENT_LIMIT, 0, 0);
		left1.configGetSupplyCurrentLimit(currentConfig);
		right1.configGetSupplyCurrentLimit(currentConfig);

        PhoenixUtil.checkError(left1.setSelectedSensorPosition(0), "drive: failed to zero left encoder");
        PhoenixUtil.checkError(right1.setSelectedSensorPosition(0), "drive: failed to zero right encoder");

        // gyro.configFactoryDefault();
        // gyro.setFusedHeading(0 /* degrees */);
    }

    public Rotation2d heading() {
        // return Rotation2d.fromDegrees(gyro.getFusedHeading());
        return Rotation2d.fromDegrees(0.0);
    }

    public DifferentialDriveWheelSpeeds speeds() {
        return new DifferentialDriveWheelSpeeds(// CONVERSION FROM ENCODER TO LEFT AND RIGHT SPEEDS (METERS PER SECOND):
                left1.getSelectedSensorVelocity() // (raw sensor units) per 100 ms
                        / Constants.kFalconTicksToRevs // (motor shaft revolutions) per 100 ms
                        / Constants.Drive.kGearRatio // (wheel shaft revolutions) per 100 ms
                        * 2 * Math.PI * Constants.Drive.kWheelRadiusMeters // meters (wheel circum) per 100 ms
                        * 10, // meters per 10(100ms) => second
                right1.getSelectedSensorVelocity() // (raw sensor units) per 100 ms
                        / Constants.kFalconTicksToRevs // (motor shaft revolutions) per 100 ms
                        / Constants.Drive.kGearRatio // (wheel shaft revolutions) per 100 ms
                        * 2 * Math.PI * Constants.Drive.kWheelRadiusMeters // meters (wheel circum) per 100 ms
                        * 10 // meters per 10(100ms) => second
        );
    }

    public double leftVoltage() {
        return left1.getMotorOutputVoltage();
    }

    public double rightVoltage() {
        return right1.getMotorOutputVoltage();
    }

    public double leftDistanceMeters() {
        return left1.getSelectedSensorPosition() // value of raw sensor units
                / Constants.kFalconTicksToRevs // value of motor shaft revolutions "elapsed"
                / Constants.Drive.kGearRatio // value of wheel shaft revolutions "elapsed"
                * 2 * Math.PI * Constants.Drive.kWheelRadiusMeters; // value of meters "elapsed" (turning wheel
                                                                    // revolutions into meters via circumference)
    }

    public double rightDistanceMeters() {
        return right1.getSelectedSensorPosition() // value of raw sensor units
                / Constants.kFalconTicksToRevs // value of motor shaft revolutions "elapsed"
                / Constants.Drive.kGearRatio // value of wheel shaft revolutions "elapsed"
                * 2 * Math.PI * Constants.Drive.kWheelRadiusMeters; // value of meters "elapsed" (turning wheel
                                                                    // revolutions into meters via circumference)
    }

    public DifferentialDriveKinematics kinematics() {
        return kinematics;
    }

    public Pose2d currentPose() {
        return currentPose;
    }

    public SimpleMotorFeedforward ff() {
        return characterizedFeedforward;
    }

    public PIDController leftPIDController() {
        return left;
    }

    public PIDController rightPIDController() {
        return right;
    }

    // OUTPUT-WRITING METHODS

    /**
     * Drive with given percent-outputs [-1.0 .. 1.0].
     * 
     * @param leftSpeed  Desired left percent-output.
     * @param rightSpeed Desired right percent-output.
     */
    public void setOutputPercent(double leftSpeed, double rightSpeed) {
        left1.set(TalonFXControlMode.PercentOutput, leftSpeed);
        right1.set(TalonFXControlMode.PercentOutput, rightSpeed);
    }

    /**
     * Compensates for current battery voltage to set a "nominal output" of two
     * voltages.
     * 
     * @param leftVoltage  The desired/nominal voltage for the left side of the
     *                     drive.
     * @param rightVoltage The desired/nominal voltage for the right side of the
     *                     drive.
     */
    public void setOutputVolts(double leftVoltage, double rightVoltage) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        left1.set(TalonFXControlMode.PercentOutput, leftVoltage / batteryVoltage);
        right1.set(TalonFXControlMode.PercentOutput, rightVoltage / batteryVoltage);
    }

    /**
     * Resets the robot position to the origin, while maintaining heading.
     */
    public void reset() {
        odometry.resetPosition(new Pose2d(), heading());
    }

    /**
     * This method is called every tick to update the robot pose estimation.
     */
    @Override
    public void periodic() {
        var heading = heading();
        var leftDistanceMeters = leftDistanceMeters();
        var rightDistanceMeters = rightDistanceMeters();

        // DEBUG VALUES
        // SmartDashboard.putNumber("drive/heading", heading.getDegrees());
        // SmartDashboard.putNumber("drive/leftVoltage (v)", leftVoltage());
        // SmartDashboard.putNumber("drive/rightVoltage (v)", rightVoltage());
        // SmartDashboard.putNumber("drive/leftDistance (m)", leftDistanceMeters);
        // SmartDashboard.putNumber("drive/rightDistance (m)", rightDistanceMeters);
        // SmartDashboard.putString("drive/wheelSpeeds", speeds().toString());
        // SmartDashboard.putString("drive/heading", heading().toString());
        // SmartDashboard.putNumber("drive/rightEncoderP", right1.getSelectedSensorPosition());
        // SmartDashboard.putNumber("drive/leftEncoderP", left1.getSelectedSensorPosition());
        // SmartDashboard.putNumber("drive/left1PctOut", left1.getMotorOutputPercent());
        // SmartDashboard.putNumber("drive/left2PctOut", left2.getMotorOutputPercent());
        // SmartDashboard.putNumber("drive/right1PctOut", right1.getMotorOutputPercent());
        // SmartDashboard.putNumber("drive/right2PctOut", right2.getMotorOutputPercent());

        currentPose = odometry.update(heading, leftDistanceMeters, rightDistanceMeters);
        PoseLoggingTable.getInstance().publishRobotPose(currentPose);
    }
}
