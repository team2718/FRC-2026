package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.turret.TurretShoot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    XboxController driverController = new XboxController(0);

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private final TurretSubsystem m_turret = new TurretSubsystem();

    private final TurretShoot shoot = new TurretShoot(m_turret, 0.5);

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
            () -> driverController.getLeftY() * -1,
            () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(OperatorConstants.SPEED_MULTIPLIER)
            .scaleRotation(OperatorConstants.ROTATION_MULTIPLIER);

    private SendableChooser<String> autoChooser = new SendableChooser<String>();
            
    public RobotContainer() {
        swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));

        autoChooser.setDefaultOption("An Auto", "An Auto");
        autoChooser.addOption("Another Auto", "Another Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
    }

    public Command getAutonomousCommand() {
      return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}