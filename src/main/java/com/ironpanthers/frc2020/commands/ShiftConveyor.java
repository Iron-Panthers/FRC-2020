package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShiftConveyor extends CommandBase {

    public enum Direction {
        kIn, kOut
    }

    private final Direction direction;
    private final ConveyorBelt conveyor;
    private int targetEncoderPosition;

    /**
     * Create a new ShiftConveyor command to shift the conveyor stack by one
     * ball-length in a direction.
     * <p>
     * ShiftConveyor will terminate upon being scheduled if inwards is true and the
     * conveyor is attempting to shift inwards.
     * 
     * @param inwards  If true, the conveyor should shift a ball-length inwards. If
     *                 false, the conveyor should exhaust.
     * @param conveyor The instance of the ConveyorBelt subsystem for the command to
     *                 use.
     */
    public ShiftConveyor(Direction direction, ConveyorBelt conveyor) {
        this.direction = direction;
        this.conveyor = conveyor;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (direction == Direction.kIn)
            if (conveyor.conveyorFull())
                cancel();

        final var encoderStartTicks = conveyor.getPosition();
        targetEncoderPosition = direction == Direction.kIn ? encoderStartTicks - Constants.Conveyor.TICKS_PREP_DISTANCE
                : encoderStartTicks + Constants.Conveyor.TICKS_PREP_DISTANCE;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        conveyor.setPosition(targetEncoderPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (direction == Direction.kIn) {
            return conveyor.getPosition() <= targetEncoderPosition + Constants.Conveyor.TICK_ERROR_TOLLERANCE;
        } else {
            return conveyor.getPosition() >= targetEncoderPosition - Constants.Conveyor.TICK_ERROR_TOLLERANCE;
        }
    }
}
