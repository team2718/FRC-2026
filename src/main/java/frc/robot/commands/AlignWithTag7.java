package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithTag7  extends Command{

    SwerveSubsystem swerve;
    

    public AlignWithTag7(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

    }


    @Override
    public void execute() {
        swerve.driveFieldOriented(new ChassisSpeeds(0.4,0.2,0));
        
        
    }
    
}
