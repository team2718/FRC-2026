package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretShoot extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speedcontrol;

    public TurretShoot(TurretSubsystem shooter, double speedcontrol) {
        this.shooter = shooter;

        this.speedcontrol = speedcontrol;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //shooter.setShooterSpeedRPM(speed); Multiply by 13 as a placeholder to convert the target velocity to rpm (Previous number was 3125)
        shooter.setShooterSpeedRPM(shooter.targetShooterSpeed() * 13 * speedcontrol);
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