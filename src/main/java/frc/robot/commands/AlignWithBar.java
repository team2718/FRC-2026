package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PossumUtils;

public class AlignWithBar  extends Command{

    SwerveSubsystem swerve;
    SwerveInputStream swerveInput;
    private final Translation2d barFrontLocation = new Translation2d(2.5, 5);
    

    public AlignWithBar(SwerveSubsystem swerve, SwerveInputStream swerveInput) {
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        addRequirements(swerve);

    }


    @Override
    public void execute() {
        double angleFromTag15 = barFrontLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double turnSpeed = PossumUtils.getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(), angleFromTag15) * 0.05;

        turnSpeed = PossumUtils.clamp(-0.8, 0.8, turnSpeed);

        swerve.driveFieldOriented(new ChassisSpeeds(swerveInput.get().vxMetersPerSecond, swerveInput.get().vyMetersPerSecond, turnSpeed));
    }
    
}
