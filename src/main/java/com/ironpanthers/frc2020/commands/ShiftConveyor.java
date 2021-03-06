package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.Constants;
import com.ironpanthers.frc2020.commands.arm.ArmToTarget;
import com.ironpanthers.frc2020.commands.shooter.ShooterSequence;
import com.ironpanthers.frc2020.subsystems.Arm;
import com.ironpanthers.frc2020.subsystems.ConveyorBelt;
import com.ironpanthers.frc2020.subsystems.Shooter;
import com.ironpanthers.frc2020.util.LightMode;
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
    private boolean isOuttake;
    private Shooter shooter;
    private LimelightWrapper lWrapper;
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
   
    public ShiftConveyor(Direction direction, ConveyorBelt conveyor,Shooter shooter) {
        this.direction = direction;
        this.conveyor = conveyor;
        this.shooter = shooter;
        isShoot = false;
        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(conveyor);
    }

    public ShiftConveyor(Direction direction, ConveyorBelt conveyor, Shooter shooter, int threshold,
            LimelightWrapper lWrapper, Arm arm, int velocity) {
        this.direction = direction;
        this.conveyor = conveyor;
        isShoot = true;
        this.shooter = shooter;
        this.threshold = threshold;
        this.lWrapper = lWrapper;
        shooter.velocity = velocity;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(conveyor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        final var encoderStartTicks = conveyor.getPosition();
        if (direction == Direction.kIn && conveyor.ballsHeld == 2) targetEncoderPosition += Constants.Conveyor.kSecondBallModifier;
        if (direction == Direction.kIn && conveyor.ballsHeld == 3) targetEncoderPosition += Constants.Conveyor.kThirdBallModifier;
        if (direction == Direction.kIn && conveyor.ballsHeld == 4) targetEncoderPosition += Constants.Conveyor.kFourthBallModifier;

        targetEncoderPosition = direction == Direction.kIn
                ? encoderStartTicks - Constants.Conveyor.kShiftEncoderDistance
                : encoderStartTicks + Constants.Conveyor.kShiftEncoderDistance;

        if (direction == Direction.kIn) {
            if (conveyor.ballsHeld >= 5 && conveyor.lastBallRan) {
                cancel();
            }  else if (conveyor.ballsHeld == 5 && !conveyor.lastBallRan) {
                targetEncoderPosition -= Constants.Conveyor.kShiftEncoderDistanceLast;
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (direction == Direction.kIn) {
            if (conveyor.ballsHeld >= 5 && conveyor.lastBallRan) {
                cancel();
            }
        }
        conveyor.setPosition(targetEncoderPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        if (!conveyor.lastBallRan && conveyor.ballsHeld == 5) {
            conveyor.lastBallRan = true;
        }
        if (isShoot) {
            conveyor.ballsHeld--;
            conveyor.lastBallRan = false;
            if (conveyor.ballsHeld > 0) {
                CommandScheduler.getInstance()
                        .schedule(new ShooterSequence(shooter, conveyor, shooter.velocity, threshold, lWrapper));
            } else {
                shooter.stopShooter();
                lWrapper.setLightMode(LightMode.OFF);
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Util.epsilonEquals(conveyor.getPosition(), targetEncoderPosition,
                Constants.Conveyor.kPositionErrorTolerance)
                || (conveyor.conveyorFull() && conveyor.lastBallRan && direction == Direction.kIn);
    }
}
