package frc.robot.commands.turret;

import frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroHood extends Command {
    private TurretSubsystem turret;

    public ZeroHood(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Set a small negative speed
        turret.setHoodSpeedUnchecked(-0.2);
    }

    @Override
    public boolean isFinished() {
        // Check if the current is near the stall limit
        return (turret.getTurretHoodCurrent().in(Amps) >= 8);
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor and reset the encoder position to zero
        turret.stopHood();
        turret.resetHoodPosition();
    }
}
