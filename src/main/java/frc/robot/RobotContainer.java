package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignWithHubFront;
import frc.robot.commands.climber.ClimbToLevel;
import frc.robot.commands.climber.ExtendHook;
import frc.robot.commands.climber.RetractHook;
import frc.robot.commands.indexer.SpinIndexerForeward;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunOuttake;
import frc.robot.commands.indexer.SpinIndexerBackward;
import frc.robot.commands.turret.TurretShoot;
import frc.robot.commands.turret.TurretToHub;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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

    private final LEDSubsystem m_led = new LEDSubsystem();

    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();

    private final TurretShoot turretShoot = new TurretShoot(m_turret, 1);
    private final TurretToHub turretToHub = new TurretToHub(m_turret, 0.5);

    private final SpinIndexerForeward spindexerForeward = new SpinIndexerForeward(m_indexer, 1);
    private final SpinIndexerBackward spindexerBackward = new SpinIndexerBackward(m_indexer, 1);

    private final RunIntake runIntake = new RunIntake(m_intake, 0.5);
    private final RunOuttake runOuttake = new RunOuttake(m_intake, 0.5);

    private final ClimbToLevel climbToLevel1 = new ClimbToLevel(m_climber, 1);
    private final ClimbToLevel climbToLevel2 = new ClimbToLevel(m_climber, 2);
    private final ClimbToLevel climbToLevel3 = new ClimbToLevel(m_climber, 3);
    private final ExtendHook extendHook = new ExtendHook();
    private final RetractHook retractHook = new RetractHook();

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

        //Left Trigger: Spins the intake wheel foreward, along with the indexer
        driverController.leftTrigger().whileTrue(runIntake);
        driverController.leftTrigger().whileTrue(spindexerForeward);
        
        //Left Bumper: Spins the intake wheel backward
        driverController.leftBumper().whileTrue(runOuttake);
        driverController.leftTrigger().whileTrue(spindexerBackward);

        //Right Trigger: Spins the shooter wheel while holding down
        driverController.rightTrigger().whileTrue(turretShoot);
        driverController.leftTrigger().whileTrue(spindexerForeward);
        
        //Right Bumper: Sets the turret to face a specific direction (Pointing toward the hub, or whatever specified) and setting the hood
        driverController.rightBumper().onTrue(turretToHub);
        //(Concept) Left Trigger: Sets intake setup to intake position, or starting position depending on where it is

        //D-Pad Controls Climbing
        if (climbToLevel1.isFinished()) {
            driverController.povLeft().onTrue(climbToLevel1);
        }
        if (climbToLevel2.isFinished()) {
            driverController.povUp().onTrue(climbToLevel2);
        }
        if (climbToLevel3.isFinished()) {
            driverController.povRight().onTrue(climbToLevel3);
        }
        if (retractHook.isFinished()) {
            driverController.povDown().onTrue(retractHook);
        }



        driverController.rightTrigger().onTrue(
            m_led.setLEDState(LEDState.SHOOTER)
        );

        driverController.rightTrigger().onFalse(
            m_led.setLEDState(LEDState.RAINBOW)
        );
    }


    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
        vision.updateSwervePoseFromVision(swerve.getSwerveDrive());
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}