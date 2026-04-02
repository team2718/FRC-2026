package frc.robot.commands.turret;

import frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroTurret extends Command {
    private TurretSubsystem turret;

    public ZeroTurret(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTurretNoLimits();
    }

    @Override
    public void execute() {
        // Set a small negative speed
        turret.setTurretSpeedUnchecked(-0.1);
    }

    @Override
    public boolean isFinished() {
        // Check if the current is near the stall limit
        return (turret.getTurretSpinnerCurrent().in(Amps) >= 20);
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor and reset the encoder position to zero
        turret.stopTurret();
        turret.setTurretYesLimits();
        turret.resetTurretPosition();
    }
}
