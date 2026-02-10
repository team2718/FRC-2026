package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurretToHub extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speed;

    public TurretToHub(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
      
        this.speed = speed;
    }

    public static double getWrappedAngleDifference(double source, double target) {
        double diff = (target - source) % 360;

        if (diff >= 180) {
            diff -= 360;
        }
        else if (diff <= -180) {
            diff += 360;
        }

        return diff;
    }

    @Override
    public void initialize() {}

    //Spins the turret and the hood to their respective target positions when activated
    @Override
    public void execute() {  

        // if (shooter.getTurretPosition() >= -170 && shooter.getTurretPosition() <= 170) {
        //     shooter.setTurretPosition(speed * (targetTurretPosition - shooter.getTurretPosition()));
        // } else {
        //     shooter.setTurretPosition(0);
        // }

        shooter.setHoodToAngle(shooter.targetShooterSpeed());

    }

@Override
public void end(boolean interuppted) {

}

    @Override
    public boolean isFinished() {
        return true;
    }
}
