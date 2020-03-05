package com.ironpanthers.frc2020.commands.drive;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ironpanthers.frc2020.subsystems.Drive;
import com.ironpanthers.util.Deadband;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualDriveCommand extends CommandBase {
    private final DoubleSupplier forward, turn;
    private final Trigger reverseTrigger;
    private final Drive drive;

    public ManualDriveCommand(DoubleSupplier forward, DoubleSupplier turn, Trigger reverseTrigger, Drive drive) {
        addRequirements(drive);
        this.forward = forward;
        this.turn = turn;
        this.reverseTrigger = reverseTrigger;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("MANUAL DRIVE CONTROL AVAILABLE", false);
    }

    @Override
    public void execute() {
        final var y = 0-Deadband.apply(forward.getAsDouble(), 0.1);
        final var direction = reverseTrigger.get();
        final var x = direction ? 0-Deadband.apply(turn.getAsDouble(), 0.1) : Deadband.apply(turn.getAsDouble(), 0.1);

        final var xPowd = Math.copySign(Math.pow(Math.abs(x), 2.75), x);

        final var leftOutputUnscaled = (direction ? y + xPowd : -y - xPowd);
        final var rightOutputUnscaled = (direction ? y - xPowd : -y + xPowd);

        final var normalizedOutputs = normalizePercents(leftOutputUnscaled + skim(rightOutputUnscaled), rightOutputUnscaled + skim(leftOutputUnscaled));

        drive.setOutputPercent(normalizedOutputs[0], normalizedOutputs[1]);
        SmartDashboard.putNumberArray("drive/manual/outputs", normalizedOutputs);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setOutputPercent(0, 0);
    }

    /**
     * Normalizes values such that the largest value has a magnitude of |1.0|.
     * 
     * @param values Unscaled values.
     * @return The original values, where all values have been divided by the
     *         largest absolute value in the set such that all items are in [-1.0,
     *         1.0].
     */
    private double[] normalizePercents(double... values) {
        final var greatestMagnitude = Arrays.stream(values).map(it -> Math.abs(it)).max().getAsDouble();
        if (greatestMagnitude <= 1.0) {
            return values; // nothing was greater than 1
        }
        var ret = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            ret[i] = values[i] / greatestMagnitude;
        }
        return ret;
    }

    private double skim(double v) {
        // Turn gain can be substituted with the slider value on the joystick, but 1.0
        // is preferred by our drivers
        double turnGain = 1.0;
        if (v > 1.0) {
            return ((v - 1.0) * turnGain);
        } else if (v < -1.0) {
            return ((v + 1.0) * turnGain);
        }
        return 0;
    }
}