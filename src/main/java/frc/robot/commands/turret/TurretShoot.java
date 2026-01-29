package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretShoot extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speed;

    public TurretShoot(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.setShooterSpeed(speed);
    }

@Override
public void end(boolean interuppted) {

}

    @Override
    public boolean isFinished() {
        return true;
    }
}