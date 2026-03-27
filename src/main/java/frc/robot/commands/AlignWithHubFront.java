package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PossumUtils;

public class AlignWithHubFront  extends Command{

    SwerveSubsystem swerve;
    SwerveInputStream swerveInput;
    private final Translation2d hubCenterLocation = new Translation2d(11.92, 4.03);
    

    public AlignWithHubFront(SwerveSubsystem swerve, SwerveInputStream swerveInput) {
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        addRequirements(swerve);

    }

    @Override
    public void execute() {
        double angleFromTag9 = hubCenterLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double turnSpeed = PossumUtils.getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(), angleFromTag9) * 0.05;

        turnSpeed = PossumUtils.clamp(-0.8, 0.8, turnSpeed);

        swerve.driveFieldOriented(new ChassisSpeeds(swerveInput.get().vxMetersPerSecond, swerveInput.get().vyMetersPerSecond, turnSpeed));
    }
    
}
