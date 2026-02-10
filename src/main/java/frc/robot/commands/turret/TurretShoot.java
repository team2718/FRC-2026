package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretShoot extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speed;

    public TurretShoot(TurretSubsystem shooter, double rpm) {
        this.shooter = shooter;

        this.speed = rpm;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setShooterSpeedRPM(speed);
    }

    @Override
    public void end(boolean interuppted) {
        shooter.setShooterSpeedRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}