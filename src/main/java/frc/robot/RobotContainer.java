package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.RebuiltMatchPeriods;
import frc.robot.Constants.RebuiltMatchPeriods.AutoWinner;
import frc.robot.Constants.RebuiltMatchPeriods.MatchPeriod;
import frc.robot.commands.climber.ExtendHook;
import frc.robot.commands.climber.RetractHook;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.indexer.SpinIndexerForeward;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.turret.TurretShoot;
import frc.robot.commands.turret.TurretToHub;
import frc.robot.commands.turret.ZeroHood;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

@Logged
public class RobotContainer {

    // ** Controllers **

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController buttonBoxController = new CommandXboxController(1);

    // ** Subsystems **

    @NotLogged
    private final SwerveSubsystem swerve = new SwerveSubsystem();

    @Logged(name="Turret")
    private final TurretSubsystem turret = new TurretSubsystem();
    @Logged(name="Indexer")
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    @Logged(name="Intake")
    private final IntakeSubsystem intake = new IntakeSubsystem();
    @Logged(name="Climber")
    private final ClimberSubsystem climber = new ClimberSubsystem();

    // private final LEDSubsystem leds = new LEDSubsystem();
    @Logged(name="Vision")
    private final VisionSubsystem vision = new VisionSubsystem();

    // ** Commands **

    private final TurretShoot turretShoot = new TurretShoot(turret, 1);
    private final TurretToHub turretToHub = new TurretToHub(turret, 0.5);

    private final SpinIndexerForeward spindexerForeward = new SpinIndexerForeward(indexer, 1);
    // private final SpinIndexerForeward spindexerBackward = new
    // SpinIndexerForeward(indexer, -1);

    private final RunIntake runIntake = new RunIntake(intake, 0.5);
    private final RunIntake runOuttake = new RunIntake(intake, -0.5);

    // private final ClimbToLevel climbToLevel1 = new ClimbToLevel(m_climber, 1);
    // private final ClimbToLevel climbToLevel2 = new ClimbToLevel(m_climber, 2);
    // private final ClimbToLevel climbToLevel3 = new ClimbToLevel(m_climber, 3);
    private final ExtendHook extendHook = new ExtendHook(climber);
    private final RetractHook retractHook = new RetractHook(climber);
    // private final SetToZero setToZero = new SetToZero(climber);

    SwerveInputStream swerveInput = swerve.getAngularVelocityFieldRelativeInputStream(driverController);
    Command swerveCommand = swerve.driveFieldOriented(swerveInput);

    private SendableChooser<String> autoChooser = new SendableChooser<String>();

    private final Timer matchTimer = new Timer();

    private boolean swerveEnabled = true;
    private boolean turretEnabled = true;
    private boolean indexerIntakeEnabled = true;
    private boolean climberEnabled = true;
    private boolean allEnabled = true;

    private boolean hasRanCalibration = true; // Set to true for testing, but should be false for comp so that it runs
                                              // at the start of the match

    public RobotContainer() {
        // Create auto chooser
        autoChooser.setDefaultOption("An Auto", "An Auto");
        autoChooser.addOption("Another Auto", "Another Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set swerve to drive with the driver's controller input by default
        swerve.setDefaultCommand(swerveCommand);

        // Setup timer

        matchTimer.reset();

        // Start match time on autonomous start
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();

            if (!hasRanCalibration) {
                CommandScheduler.getInstance().schedule(new ZeroClimber(climber));
                CommandScheduler.getInstance().schedule(new ZeroHood(turret));
                hasRanCalibration = true;
            }
        }));

        // Start match time on teleop start
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();

            if (!hasRanCalibration) {
                CommandScheduler.getInstance().schedule(new ZeroClimber(climber));
                CommandScheduler.getInstance().schedule(new ZeroHood(turret));
                hasRanCalibration = true;
            }
        }));

        // Stop match time on end of match
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.stop();
        }));

        // Configure button bindings
        configureBindings();
    }

    private void configureBindings() {

        // It wouldn't be practical for the spindexer to be mapped to a unique input, so
        // the thought is to spin it in the direction we want the fuel to go in. Subject
        // to change.

        // Left Trigger: Spins the intake wheel & spindexer foreward
        driverController.leftTrigger().whileTrue(runIntake);
        // driverController.leftTrigger().whileTrue(spindexerForeward);

        // Left Bumper: Spins the intake wheel & spindexer backward
        driverController.leftBumper().whileTrue(runOuttake);
        // driverController.leftBumper().whileTrue(spindexerBackward);

        // Right Trigger: Spins the shooter wheel & spindexer while holding down
        driverController.rightTrigger().whileTrue(turretShoot);
        driverController.rightTrigger().whileTrue(spindexerForeward);

        driverController.a().whileTrue(retractHook);
        driverController.b().whileTrue(extendHook);

        // Right Bumper: Sets the turret to face a specific direction (Pointing toward
        // the hub, or whatever specified) and setting the hood
        driverController.rightBumper().onTrue(turretToHub);
        // (Concept) Left Trigger: Sets intake setup to intake position, or starting
        // position depending on where it is

        // D-Pad Controls Climbing
        // driverController.povLeft().onTrue(climbToLevel1);

        /*
         * if (climbToLevel1.isFinished()) {
         * driverController.povLeft().onTrue(climbToLevel1);
         * }
         * if (climbToLevel2.isFinished()) {
         * driverController.povUp().onTrue(climbToLevel2);
         * }
         * if (climbToLevel3.isFinished()) {
         * driverController.povRight().onTrue(climbToLevel3);
         * }
         * if (retractHook.isFinished()) {
         * driverController.povDown().onTrue(retractHook);
         * }
         */

        // placeholder button: If the intake is at the stowed position, pressing x will
        // set it to the active position, and vise-versa
        // driverController.leftTrigger().onTrue(
        // Commands.runEnd(() -> {
        // intake.setActive();
        // intake.setIntakeVoltage(5);
        // }, () -> {
        // intake.setStowed();
        // intake.setIntakeVoltage(0);
        // }, intake));

        // driverController.rightTrigger().onTrue(
        // leds.setLEDState(LEDState.SHOOTER));

        // driverController.rightTrigger().onFalse(
        // leds.setLEDState(LEDState.RAINBOW));
    }

    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
        vision.updateSwervePoseFromVision(swerve.getSwerveDrive());

        int leftTrigger = (int) (buttonBoxController.getHID().getLeftTriggerAxis() * 32);

        swerveEnabled = (leftTrigger & 0b1) == 0;
        turretEnabled = (leftTrigger & 0b10) == 0;
        indexerIntakeEnabled = (leftTrigger & 0b100) == 0;
        climberEnabled = (leftTrigger & 0b1000) == 0;
        allEnabled = (leftTrigger & 0b10000) == 0;

        if (!allEnabled) {
            swerveEnabled = false;
            turretEnabled = false;
            indexerIntakeEnabled = false;
            climberEnabled = false;
        }

        SmartDashboard.putBoolean("Enabled/Swerve", swerveEnabled);
        SmartDashboard.putBoolean("Enabled/Turret", turretEnabled);
        SmartDashboard.putBoolean("Enabled/IndexerIntake", indexerIntakeEnabled);
        SmartDashboard.putBoolean("Enabled/Climber", climberEnabled);

        swerve.setEnabled(swerveEnabled);
        turret.setEnabled(turretEnabled);
        indexer.setEnabled(indexerIntakeEnabled);
        intake.setEnabled(indexerIntakeEnabled);
        climber.setEnabled(climberEnabled);

        updateShuffleboardTimers();
    }

    private void updateShuffleboardTimers() {
        if (DriverStation.isDisabled()) {
            SmartDashboard.putNumber("Match Time", 0);
            SmartDashboard.putString("Match Period", "Disabled");
            SmartDashboard.putNumber("Time Left in Period", 0);
            return;
        }

        if (DriverStation.isAutonomous()) {
            SmartDashboard.putNumber("Match Time", RebuiltMatchPeriods.AUTONOMOUS_DURATION - matchTimer.get());
            SmartDashboard.putString("Match Period", "Autonomous");
            SmartDashboard.putNumber("Time Left in Period", RebuiltMatchPeriods.AUTONOMOUS_DURATION - matchTimer.get());
        } else if (DriverStation.isTeleop()) {
            SmartDashboard.putNumber("Match Time", (2 * 60 + 20) - matchTimer.get());
            SmartDashboard.putNumber("Time Left in Period",
                    RebuiltMatchPeriods.getTimeLeftInCurrentPeriod(matchTimer.get()));
            AutoWinner autoWinner = AutoWinner.fromGameData(DriverStation.getGameSpecificMessage());
            MatchPeriod period = Constants.RebuiltMatchPeriods.getTeleopPeriodFromTime(matchTimer.get());

            if (autoWinner == AutoWinner.UNKNOWN) {
                SmartDashboard.putString("Match Period", period.toString());
                return;
            }

            if (period == MatchPeriod.TRANSITION_SHIFT) {
                SmartDashboard.putString("Match Period",
                        period.toString() + "\n" + autoWinner.opposite() + " up first!");
            } else if (period == MatchPeriod.SHIFT_1 || period == MatchPeriod.SHIFT_3) {
                SmartDashboard.putString("Match Period", period.toString() + "\n" + autoWinner.opposite() + " active!");
            } else if (period == MatchPeriod.SHIFT_2 || period == MatchPeriod.SHIFT_4) {
                SmartDashboard.putString("Match Period", period.toString() + "\n" + autoWinner + " active!");
            } else {
                SmartDashboard.putString("Match Period", period.toString());
            }

        } else {
            SmartDashboard.putNumber("Match Time", 0);
            SmartDashboard.putString("Match Period", "Testing");
            SmartDashboard.putNumber("Time Left in Period", 0);
        }
    }

    public Command getAutonomousCommand() {
        // return null;
        return swerve.getAutonomousCommand(autoChooser.getSelected());
    }

}