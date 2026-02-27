package frc.robot.commands.turret;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroHood extends Command {
    private TurretSubsystem turret;

    public ZeroHood(TurretSubsystem turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Set a small negative voltage
        turret.setTurretHood(-0.2);
    }

    @Override
    public boolean isFinished() {
        // Check if the current is near the stall limit
        return (turret.getTurretHoodCurrent() >= 8);
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor and reset the encoder position to zero
        turret.setTurretHood(0);
        turret.resetHoodPosition();
    }
}
