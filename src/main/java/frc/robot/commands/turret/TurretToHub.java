package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretToHub extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speed;

    public TurretToHub(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    //Spins the turret to position 180 (placeholder) when activated
    @Override
    public void execute() {
        if (shooter.getTurretPosition() >= 0 && shooter.getTurretPosition() <= 360) {
            shooter.setTurretSpin(speed * (180 - shooter.getTurretPosition())); //180 = desired angle
        } else {
            shooter.setTurretSpin(0);
        }
    }

@Override
public void end(boolean interuppted) {

}

    @Override
    public boolean isFinished() {
        return true;
    }
}