package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.turret.TurretShoot;
//import frc.robot.commands.turret.TurretToHub;
import frc.robot.commands.AlignWithHubFront;
//import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    //XboxController driverController = new XboxController(0);

    CommandXboxController driverController = new CommandXboxController(0);

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    // private final TurretSubsystem m_turret = new TurretSubsystem();
    // private final IntakeSubsystem m_intake = new IntakeSubsystem();

    // private final TurretShoot turretShoot = new TurretShoot(m_turret, 0.5);
    // private final TurretToHub turretToHub = new TurretToHub(m_turret, 0.5);
    // private final RunIntake runIntake = new RunIntake(m_intake, 0.5);
    // private final RunOuttake runOuttake = new RunOuttake(m_intake, 0.5);

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

        configureBindings();
        driverController.b().whileTrue(new AlignWithHubFront(swerve, driveAngularVelocityRobotRelative));
    }

    private void configureBindings() {

        //Left Trigger: Spins the intake wheel when the setup is at the ending position
        // driverController.leftTrigger().onTrue(runIntake);
        
        //Left Bumper: Spins the intake wheel backwards when the setup is at the ending position
        // driverController.leftBumper().onTrue(runOuttake);

        //Right Trigger: Spins the shooter wheel while holding down
        // driverController.rightTrigger().onTrue(turretShoot);
        
        //Right Bumper: Sets the turret to face a specific direction (Pointing toward the hub, or whatever specified) and setting the hood
        //driverController.rightBumper().onTrue(turretToHub);
        //(Concept) Left Trigger: Sets intake setup to intake position, or starting position depending on where it is

        /*
        if (intakeAtStartingPos) {
            driverController.leftBumper().onTrue(setIntakePos);
        } else if (intakeAtEndingPos) {
            driverController.leftBumper().onTrue(unsetIntakePos);
        } else {
            driverController.leftBumper().onTrue(stopIntake);
        }
        */

    }

    
    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
        vision.updateSwervePoseFromVision(swerve.getSwerveDrive());
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}