package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurretToHub extends Command {
    private final TurretSubsystem shooter;
    // private final LEDSubsystem LEDs;
    private double speed;

    private double targetTurretPosition;
    private double targetHoodPosition;

    SwerveSubsystem swerve;
    private final Translation2d hubCenterLocation = new Translation2d(11.92, 4.03);

    public TurretToHub(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
      
        this.speed = speed;

        this.swerve = swerve;

        double angleFromTag9 = hubCenterLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double distanceToTag9 = Math.sqrt(Math.abs(swerve.getPose().getX() - hubCenterLocation.getX()) + Math.abs(swerve.getPose().getY() - hubCenterLocation.getY()));

        //set turret target position
        targetTurretPosition = getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(), angleFromTag9);

        //set turret hood target position (Rudimentary calculation)
        if (distanceToTag9 < 1) {
            targetHoodPosition = 0;
        } else {
            targetHoodPosition = 30 - (30 / distanceToTag9);
        }
    }

    public static double getWrappedAngleDifference(double source, double target) {
        double diff = (target - source) % 360;

        if (diff > 180) {
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

        if (shooter.getTurretPosition() >= -170 && shooter.getTurretPosition() <= 170) {
            shooter.setTurretSpin(speed * (targetTurretPosition - shooter.getTurretPosition()));
        } else {
            shooter.setTurretSpin(0);
        }

        if (shooter.getTurretHood() >= 0 && shooter.getTurretHood() <= 360) {
            shooter.setTurretHood(speed * (targetHoodPosition - shooter.getTurretHood()));
        } else {
            shooter.setTurretHood(0);
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