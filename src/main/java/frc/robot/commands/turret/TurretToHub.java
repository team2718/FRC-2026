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

    private double turretDistanceToRobotCenter = 0.5;
    private double turretDegreeFromRobotCenter = 40;

    SwerveSubsystem swerve;
    private final Translation2d hubCenterLocation = new Translation2d(11.92, 4.03);

    public TurretToHub(TurretSubsystem shooter, double speed) {
        this.shooter = shooter;
      
        this.speed = speed;

        this.swerve = swerve;

        double robotAngleFromTag9 = hubCenterLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double robotDistanceToTag9 = Math.sqrt(Math.pow(swerve.getPose().getX() - hubCenterLocation.getX(), 2) + Math.pow(swerve.getPose().getY() - hubCenterLocation.getY(), 2));


        //Calculates the position of the turret on the field by taking the turret's distance and angle (relative to the where the robot's facing) from the center point of the robot
        double turretPositionX = swerve.getPose().getX() + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() + turretDegreeFromRobotCenter) * turretDistanceToRobotCenter;
        double turretPositionY = swerve.getPose().getY() + Math.sin(swerve.getPose().getTranslation().getAngle().getDegrees() + turretDegreeFromRobotCenter) * turretDistanceToRobotCenter;

        double turretAngleFromTag9 = Math.atan2(hubCenterLocation.getX() - turretPositionX, hubCenterLocation.getY() - turretPositionY);

        double turretDistanceToTag9 = Math.sqrt(Math.pow(turretPositionX - hubCenterLocation.getX(), 2) + Math.pow(turretPositionY - hubCenterLocation.getY(), 2));

        /*
        When we shoot fuel while the robot is moving, the fuel's speed moves relative to the robot's speed, throwing us off.
        To counteract this, we can calculate the time it will take for the fuel to reach the goal, and project the robot's position at that point. (current position + ( current velocity * the fuel goal time )) 
        If we point the turret in the direction of the hub based from that position instead of it's current position, it should line up.
         */

        //Projects the time it will take for the feul to reach the goal when the robot shoots (Currently a shoddy estimate, farther = longer time)
        double projectedTime = Math.sqrt(robotDistanceToTag9);

        //Projects the turret's projected location relative to the field
        double projectedTurretPositionX = swerve.getPose().getX() + (swerve.getRobotVelocity().vxMetersPerSecond * projectedTime) + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() + turretDegreeFromRobotCenter) * turretDistanceToRobotCenter;
        double projectedTurretPositionY = swerve.getPose().getY() + (swerve.getRobotVelocity().vyMetersPerSecond * projectedTime) + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() + turretDegreeFromRobotCenter) * turretDistanceToRobotCenter;

        //Projects the turret's angle from the hub
        double projectedTurretAngleFromTag9 = Math.atan2(hubCenterLocation.getX() - projectedTurretPositionX, hubCenterLocation.getY() - projectedTurretPositionY);
        //Projects the turret's distance to the hub
        double projectedTurretDistanceToTag9 = Math.sqrt(Math.pow(projectedTurretPositionX - hubCenterLocation.getX(), 2) + Math.pow(projectedTurretPositionY - hubCenterLocation.getY(), 2));

        //set turret target position
        targetTurretPosition = getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(), projectedTurretAngleFromTag9);

        //set turret hood target position (Rudimentary calculation)
        if (projectedTurretDistanceToTag9 < 1) {
            targetHoodPosition = 0;
        } else {
            targetHoodPosition = 30 - (30 / projectedTurretDistanceToTag9);
        }
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

        shooter.setHoodToAngle(30);

    }

@Override
public void end(boolean interuppted) {

}

    @Override
    public boolean isFinished() {
        return true;
    }
}
