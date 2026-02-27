package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretToHub extends Command {
    private final TurretSubsystem shooter;
    public TurretToHub(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
        addRequirements(shooter);
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

        shooter.setHoodToAngle(shooter.targetHoodAngle());

    }

@Override
public void end(boolean interuppted) {

}

    @Override
    public boolean isFinished() {
        return true;
    }
}
