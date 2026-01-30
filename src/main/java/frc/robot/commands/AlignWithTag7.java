package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithTag7  extends Command{

    SwerveSubsystem swerve;
    

    public AlignWithTag7(SwerveSubsystem swerve) {
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
        double distFromTag7X = swerve.getPose().getTranslation().getX() - VisionSubsystem.fieldLayout.getTagPose(7).get().getX();
        double distFromTag7Y = swerve.getPose().getTranslation().getY() - VisionSubsystem.fieldLayout.getTagPose(7).get().getY();
        double angleFromTag7 = swerve.getPose().getRotation().getDegrees() - 0.0; // 0.0 is the desired angle

        double xSpeed = -distFromTag7X * 5.0;
        double ySpeed = -distFromTag7Y * 5.0;
        double turnSpeed = -angleFromTag7 * 0.05;

        xSpeed = clamp(-0.8, 0.8, xSpeed);
        ySpeed = clamp(-0.8, 0.8, ySpeed);
        turnSpeed = clamp(-0.8, 0.8, turnSpeed);

        swerve.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed,turnSpeed));
    }
    
}
