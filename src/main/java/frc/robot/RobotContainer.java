package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.SetIntakeActivator;
import frc.robot.commands.intake.UnsetIntakeActivator;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    CommandXboxController driverController = new CommandXboxController(0);
    /*
    private final JoystickButton driverXboxButtonB = new JoystickButton(driverController, Constants.OI.kdriverControllerButton2);
    private final JoystickButton driverXboxleftbumper = new JoystickButton(driverController, Constants.OI.kdriverControllerButton5);
    private final JoystickButton driverXboxrightbumper = new JoystickButton(driverController, Constants.OI.kdriverControllerButton6);
    private final JoystickButton driverXboxButtonA = new JoystickButton(driverController, Constants.OI.kdriverControllerButton1);
    private final JoystickButton driverXboxButtonY = new JoystickButton(driverController, Constants.OI.kdriverControllerButton4);
    private final JoystickButton driverXboxButtonX = new JoystickButton(driverController, Constants.OI.kdriverControllerButton3);
    private final JoystickButton driverXboxButtonMinus = new JoystickButton(driverController, Constants.OI.kdriverControllerButton7);
    private final JoystickButton driverXboxButtonPlus = new JoystickButton(driverController, Constants.OI.kdriverControllerButton8);
    private final POVButton driverXboxDpad = new POVButton(driverController, 0);
    */
    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    //Intake Commands
    private final IntakeCommand runIntake = new IntakeCommand(m_intake, 0.5);
    private final SetIntakeActivator setIntakePos = new SetIntakeActivator(m_intake, 0.5);
    private final UnsetIntakeActivator unsetIntakePos = new UnsetIntakeActivator(m_intake, 0.5);
    private final UnsetIntakeActivator stopIntake = new UnsetIntakeActivator(m_intake, 0);


    private final boolean intakeAtEndingPos = m_intake.getActivatorPosition() > 117;
    private final boolean intakeAtStartingPos = m_intake.getActivatorPosition() < 3;

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

        configureBindings();
    }

    private void configureBindings() {

        //Right Bumper: Spins the intake wheel when the setup is at the ending position
        driverController.rightBumper().onTrue(runIntake);

        //Left Bumper: Sets intake setup to intake position, or starting position depending on where it is
        if (intakeAtStartingPos) {
            driverController.leftBumper().onTrue(setIntakePos);
        } else if (intakeAtEndingPos) {
            driverController.leftBumper().onTrue(unsetIntakePos);
        } else {
            driverController.leftBumper().onTrue(stopIntake);
        }
            
    }

    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
    }

    public Command getAutonomousCommand() {
      return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}