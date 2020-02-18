package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence2;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LimelightWrapper;
import com.ironpanthers.util.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ShiftConveyor extends CommandBase {

    public enum Direction {
        kIn, kOut
    }

    private final Direction direction;
    private final ConveyorBelt conveyor;
    private int targetEncoderPosition;
    private boolean isShoot;
    private Shooter shooter;
    private LimelightWrapper lWrapper;
    private int velocity;
    private int threshold;

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
        isShoot = false;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(conveyor);
    }

    public ShiftConveyor(Direction direction, ConveyorBelt conveyor, Shooter shooter, int velocity, int threshold, LimelightWrapper lWrapper) {
        this.direction = direction;
        this.conveyor = conveyor;
        isShoot = true;
        this.shooter = shooter;
        this.velocity = velocity;
        this.threshold = threshold;
        this.lWrapper = lWrapper;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (direction == Direction.kIn) {
            if (conveyor.conveyorFull())
                cancel();
        }
            
        

        final var encoderStartTicks = conveyor.getPosition();
        targetEncoderPosition = direction == Direction.kIn ? encoderStartTicks - Constants.Conveyor.kShiftEncoderDistance
                : encoderStartTicks + Constants.Conveyor.kShiftEncoderDistance;
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
        if (isShoot) {
            if (direction == Direction.kOut && conveyor.ballsHeld >= 1) {
                conveyor.ballsHeld--;
                CommandScheduler.getInstance().schedule(new ShooterSequence2(shooter, conveyor, velocity, threshold, lWrapper));
            } else if(direction == Direction.kOut && conveyor.ballsHeld == 0) {
                lWrapper.turnOffLight();
            }
        }
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(conveyor.getPosition(), targetEncoderPosition, Constants.Conveyor.kPositionErrorTolerance);
    }
}
