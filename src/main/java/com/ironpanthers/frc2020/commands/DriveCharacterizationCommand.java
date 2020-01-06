package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.subsystems.Drive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Characterizes the drivetrain when used in conjunction with
 * frc-characterization toolsuite.
 * 
 * @author Ingi Helgason
 */
public class DriveCharacterizationCommand extends CommandBase {
    /**
     * The location where the desired outputs to write can be found. Basically, the
     * frc-characterization toolsuite generates "quasistatically" increasing or
     * decreasing outputs which the user is expected to write to the subysstem, and
     * puts them here.
     */
    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    /**
     * The location where the frc-characterization toolsuite expects us to publish
     * our results. From here, the frc-characterization program stores this data,
     * which we can later use to generate mathematically-ideal characterization
     * constants for the drivebase.
     */
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    /**
     * The number array which is published to NetworkTables for the toolsuite to
     * process.
     */
    private Number[] telemetry = new Number[9];
    private final Drive drive;

    /**
     * Create a new drive characterization command.
     * 
     * @param drive The instance of the drive subsystem to be used. This subsystem
     *              is marked as a requirement to the scheduler, so keep in mind
     *              that commands actively running which require this subsystem will
     *              likely be cancelled.
     */
    public DriveCharacterizationCommand(Drive drive) {
        addRequirements(drive);
        this.drive = drive;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        System.out.println("Started DriveCharacterizationCommand");
    }

    // Update autospeed, write outputs, publish telemetry to nt
    @Override
    public void execute() {
        // UPDATE AUTOSPEED
        var autospeed = autoSpeedEntry.getDouble(0.0);

        // WRITE OUTPUT
        System.out.println("Autospeed: " + autospeed);
        drive.setOutputPercent(autospeed, autospeed);

        // POPULATE DATA PACKET
        telemetry[0] = Timer.getFPGATimestamp(); // fpga timestamp
        telemetry[1] = RobotController.getBatteryVoltage(); // voltage
        telemetry[2] = autospeed; // autospeed
        telemetry[3] = drive.leftVoltage(); // left voltage
        telemetry[4] = drive.rightVoltage(); // right voltage
        telemetry[5] = drive.leftDistanceMeters(); // left pos
        telemetry[6] = drive.rightDistanceMeters(); // right pos
        var speeds = drive.speeds();
        telemetry[7] = speeds.leftMetersPerSecond; // left speed
        telemetry[8] = speeds.rightMetersPerSecond; // right speed

        // PUSH DATA
        telemetryEntry.setNumberArray(telemetry);
    }

    // Cut power to the drivetrain and indicate the command has been stopped
    @Override
    public void end(boolean interrupted) {
        drive.setOutputPercent(0, 0);
        System.out.println("Ended DriveCharacterizationCommand");
    }
}