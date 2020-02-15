package com.ironpanthers.frc2020.commands.shooter;

import com.ironpanthers.frc2020.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class StopShooter extends InstantCommand {
    private final Shooter shooter;

    /**
     * Creates a new StopShooter command.
     * <p>
     * The StopShooter command instantly gives the Shooter a PercentOutput reference
     * of zero (using the method {@link Shooter#stopShooter()}).
     * 
     * @param shooter The instance of the Shooter subsystem for the command to use.
     */
    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stopShooter();
    }
}
