package com.ironpanthers.frc2020.commands.drive;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ironpanthers.util.Dashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static com.ironpanthers.util.ErrorMessages.requireNonNullParam;

public class ReportingRAMSETECommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final boolean m_usePID;
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    /**
     * Constructs a new ReportingRAMSETECommand that, when executed, will follow the
     * provided trajectory. PID control and feedforward are handled internally, and
     * outputs are scaled -12 to 12 representing units of volts.
     *
     * <p>
     * Note: The controller will *not* set the outputVolts to zero upon completion
     * of the path - this is left to the user, since it is not appropriate for paths
     * with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param pose            A function that supplies the robot pose - use one of
     *                        the odometry classes to provide this.
     * @param controller      The RAMSETE controller used to follow the trajectory.
     * @param feedforward     The feedforward to use for the drive.
     * @param kinematics      The kinematics for the robot drivetrain.
     * @param wheelSpeeds     A function that supplies the speeds of the left and
     *                        right sides of the robot drive.
     * @param leftController  The PIDController for the left side of the robot
     *                        drive.
     * @param rightController The PIDController for the right side of the robot
     *                        drive.
     * @param outputVolts     A function that consumes the computed left and right
     *                        outputs (in volts) for the robot drive.
     * @param requirements    The subsystems to require.
     */
    public ReportingRAMSETECommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller,
            SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics,
            Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController,
            PIDController rightController, BiConsumer<Double, Double> outputVolts, Subsystem... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "ReportingRAMSETECommand");
        m_pose = requireNonNullParam(pose, "pose", "ReportingRAMSETECommand");
        m_follower = requireNonNullParam(controller, "controller", "ReportingRAMSETECommand");
        m_feedforward = feedforward;
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "ReportingRAMSETECommand");
        m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "ReportingRAMSETECommand");
        m_leftController = requireNonNullParam(leftController, "leftController", "ReportingRAMSETECommand");
        m_rightController = requireNonNullParam(rightController, "rightController", "ReportingRAMSETECommand");
        m_output = requireNonNullParam(outputVolts, "outputVolts", "ReportingRAMSETECommand");

        m_usePID = true;

        addRequirements(requirements);
    }

    /**
     * Constructs a new ReportingRAMSETECommand that, when executed, will follow the provided
     * trajectory. Performs no PID control and calculates no feedforwards; outputs
     * are the raw wheel speeds from the RAMSETE controller, and will need to be
     * converted into a usable form by the user.
     *
     * @param trajectory            The trajectory to follow.
     * @param pose                  A function that supplies the robot pose - use
     *                              one of the odometry classes to provide this.
     * @param follower              The RAMSETE follower used to follow the
     *                              trajectory.
     * @param kinematics            The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and
     *                              right wheel speeds.
     * @param requirements          The subsystems to require.
     */
    public ReportingRAMSETECommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "ReportingRAMSETECommand");
        m_pose = requireNonNullParam(pose, "pose", "ReportingRAMSETECommand");
        m_follower = requireNonNullParam(follower, "follower", "ReportingRAMSETECommand");
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "ReportingRAMSETECommand");
        m_output = requireNonNullParam(outputMetersPerSecond, "output", "ReportingRAMSETECommand");

        m_feedforward = null;
        m_speeds = null;
        m_leftController = null;
        m_rightController = null;

        m_usePID = false;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        if (m_usePID) {
            m_leftController.reset();
            m_rightController.reset();
        }
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        var trajectorySample = m_trajectory.sample(curTime);

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(m_pose.get(), trajectorySample));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint,
                    (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint,
                    (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;

        Dashboard.getInstance().publishPathSample(trajectorySample.poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}