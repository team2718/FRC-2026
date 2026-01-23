package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithHubFront  extends Command{

    SwerveSubsystem swerve;
    

    public AlignWithHubFront(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

    }

    public double clamp(double min, double max, double value) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }


    @Override
    public void execute() {
        double distFromTag9X = swerve.getPose().getTranslation().getX() - VisionSubsystem.fieldLayout.getTagPose(9).get().getX();
        double distFromTag9Y = swerve.getPose().getTranslation().getY() - VisionSubsystem.fieldLayout.getTagPose(9).get().getY();
        double angleFromTag9 = swerve.getPose().getRotation().getDegrees() - 0.0; // 0.0 is the desired angle

        double distFromTag10X = swerve.getPose().getTranslation().getX() - VisionSubsystem.fieldLayout.getTagPose(10).get().getX();
        double distFromTag10Y = swerve.getPose().getTranslation().getY() - VisionSubsystem.fieldLayout.getTagPose(10).get().getY();
        double angleFromTag10 = swerve.getPose().getRotation().getDegrees() - 0.0; // 0.0 is the desired angle

        //double xSpeed = -distFromTag9X * 5.0;
        //double ySpeed = -distFromTag9Y * 5.0;
        //double turnSpeed = -angleFromTag9 * 0.05;

        double xSpeed = -distFromTag10X * 5.0;
        double ySpeed = -distFromTag10Y * 5.0;
        double turnSpeed = -angleFromTag10 * 0.05;

        xSpeed = clamp(-0.8, 0.8, xSpeed);
        ySpeed = clamp(-0.8, 0.8, ySpeed);
        turnSpeed = clamp(-0.8, 0.8, turnSpeed);

        swerve.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed,turnSpeed));
    }
    
}
