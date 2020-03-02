package com.ironpanthers.frc2020.commands;

import com.ironpanthers.frc2020.subsystems.ConveyorBelt;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConstantVelocityConveyorExhaust extends CommandBase {
    private final double velocitySTU;
    private final ConveyorBelt conveyor;

    /**
     * Creates a new ConstantVelocityConveyorExhaust.
     */
    public ConstantVelocityConveyorExhaust(double velocitySTU, ConveyorBelt conveyor) {
        this.velocitySTU = velocitySTU;
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setVelocity(velocitySTU);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.ballsHeld == 0;
    }
}
