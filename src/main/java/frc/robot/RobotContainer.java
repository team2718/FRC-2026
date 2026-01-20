package frc.robot;

import java.io.File;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignWithTag7;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    CommandXboxController driverController = new CommandXboxController(0);

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    VisionSubsystem vision = new VisionSubsystem();

    SwerveInputStream driveAngularVelocityRobotRelative = SwerveInputStream.of(swerve.getSwerveDrive(),
            () -> driverController.getLeftY() * -1,
            () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(OperatorConstants.SPEED_MULTIPLIER)
            .scaleRotation(OperatorConstants.ROTATION_MULTIPLIER)
            .allianceRelativeControl(false)
            .robotRelative(false);

    SwerveInputStream driveDirectAngleFieldRelative = driveAngularVelocityRobotRelative.copy()
            .withControllerHeadingAxis(driverController::getRightX, driverController::getRightY)
            .headingWhile(true)
            .robotRelative(false)
            .allianceRelativeControl(true);

    private SendableChooser<String> autoChooser = new SendableChooser<String>();

    public RobotContainer() {
        swerve.setDefaultCommand(swerve.drive(driveAngularVelocityRobotRelative));

        driverController.a().onTrue(Commands.runOnce(swerve::zeroGyro));

        autoChooser.setDefaultOption("An Auto", "An Auto");
        autoChooser.addOption("Another Auto", "Another Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        driverController.b().whileTrue(new AlignWithTag7(swerve));
    }

    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
        vision.updatePoseFromTags(swerve.getSwerveDrive());
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}