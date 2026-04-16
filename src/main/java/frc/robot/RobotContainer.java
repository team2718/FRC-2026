package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.RebuiltMatchPeriods;
import frc.robot.Constants.RebuiltMatchPeriods.AutoWinner;
import frc.robot.Constants.RebuiltMatchPeriods.MatchPeriod;
import frc.robot.commands.indexer.SpinIndexerForeward;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.intake.OscillateIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.turret.AimAndPrespinTurret;
import frc.robot.commands.turret.TurretToHub;
import frc.robot.commands.turret.TurretToHubAuto;
import frc.robot.commands.turret.ZeroHood;
import frc.robot.commands.turret.ZeroTurret;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.utils.BetterAutoChooser;
import swervelib.SwerveInputStream;

@Logged
public class RobotContainer {

    // ** Controllers **

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController buttonBoxController = new CommandXboxController(1);

    // ** Subsystems **

    @NotLogged
    public final SwerveSubsystem swerve = new SwerveSubsystem();

    @Logged(name = "Turret")
    private final TurretSubsystem turret = new TurretSubsystem();
    @Logged(name = "Indexer")
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    @Logged(name = "Intake")
    private final IntakeSubsystem intake = new IntakeSubsystem();
    @Logged(name = "IntakeArm")
    private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();

    // @Logged(name = "Climber")
    // private final ClimberSubsystem climber = new ClimberSubsystem();

    // private final LEDSubsystem leds = new LEDSubsystem();
    @Logged(name = "Vision")
    private final VisionSubsystem vision = new VisionSubsystem();

    // @NotLogged
    // private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    // ** Commands **

    private Command pathPlannerAutoCommand; // This will hold the currently selected auto command from the chooser

    private final SpinIndexerForeward spindexerBackward = new SpinIndexerForeward(indexer, -8);
    private final RunIntake runIntake = new RunIntake(intake, 0.75);
    private final RunIntake runOutake = new RunIntake(intake, -0.5);
    // private final ExtendHook extendHook = new ExtendHook(climber);
    // private final RetractHook retractHook = new RetractHook(climber);
    // private final SpinUpTurret spinUpTurret = new SpinUpTurret(turret, 1800);

    private final LEDSubsystem led = new LEDSubsystem();

    private final SwerveInputStream swerveInput = swerve.getAngularVelocityFieldRelativeInputStream(driverController);
    private final Command swerveCommand = swerve.driveFieldOriented(swerveInput);

    private final TurretToHub turretToHub = new TurretToHub(turret, swerve, indexer, intake, swerveInput, led);

    private final SendableChooser<Command> autoChooser = BetterAutoChooser.buildAutoChooser();

    private final Timer matchTimer = new Timer();

    private final Timer globalTimer = new Timer();

    private final Command shootInAuto = new TurretToHubAuto(turret, swerve, indexer, intake, led);

    private boolean swerveEnabled = true;
    private boolean turretEnabled = true;
    private boolean indexerIntakeEnabled = true;
    private boolean climberEnabled = true;
    private boolean allEnabled = true;

    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Misc/Robot Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Misc/Turret Pose", Pose2d.struct).publish();

    private boolean hasRanCalibration = false; // Set to true for testing, but should be false for comp so that it runs
                                               // at the start of the match

    public RobotContainer() {
        swerve.setDefaultCommand(swerve.driveFieldOriented(swerveInput));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set swerve to drive with the driver's controller input by default
        swerve.setDefaultCommand(swerveCommand);

        // led.setDefaultCommand(Commands.run(() -> {led.setLEDState(LEDState.YELLOW);}, led));

        // Setup timer

        matchTimer.reset();
        globalTimer.start();

        // Start match time on autonomous start
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();
        }));

        // Start match time on teleop start
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();

            // Run calibration in teleop in case it failed in auto
            if (!hasRanCalibration) {
                // CommandScheduler.getInstance().schedule(new ZeroClimber(climber));
                CommandScheduler.getInstance().schedule(new ZeroHood(turret));
                CommandScheduler.getInstance().schedule(new ZeroTurret(turret));
                hasRanCalibration = true;
            }
        }));

        // Stop match time on end of match
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.stop();
        }));

        // Buttons to manually zero stuff as needed
        SmartDashboard.putData("Commands/Zero Hood", new ZeroHood(turret));
        SmartDashboard.putData("Commands/Zero Turret", new ZeroTurret(turret));
        // SmartDashboard.putData("Commands/Zero Climber", new ZeroClimber(climber));

        NamedCommands.registerCommand("RunIntake",
                new AutoRunIntake(intake, intakeArm, 0.75));

        NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intake.setIntakeSpeed(0), intake));

        // NamedCommands.registerCommand("SpinUpTurret", spinUpTurret);

        NamedCommands.registerCommand("SpinIndexerForward",
                new SpinIndexerForeward(indexer, .5));

        NamedCommands.registerCommand("StartShooting", shootInAuto);

        NamedCommands.registerCommand("StopShooting", Commands.runOnce(() -> {
            turret.stopShooter();
            indexer.stopIndexing();
            intakeArm.setActive();
        }, turret, indexer, intakeArm));

        NamedCommands.registerCommand("OscillateIntake", new OscillateIntake(intakeArm));

        NamedCommands.registerCommand("SpinTo90", new AimAndPrespinTurret(turret, 90));
        NamedCommands.registerCommand("SpinTo180", new AimAndPrespinTurret(turret, 180));
        NamedCommands.registerCommand("SpinTo270", new AimAndPrespinTurret(turret, 270));

        // NamedCommands.registerCommand("ExtendHook",
        // new ExtendHook(climber));

        // NamedCommands.registerCommand("RetractHook",
        // new RetractHook(climber));

        NamedCommands.registerCommand("TurretToHub",
                new ParallelDeadlineGroup(
                        new WaitCommand(5),
                        new TurretToHub(turret, swerve, indexer, intake, swerveInput, led)));

        // Register turret to hub for 1 to 9 seconds
        for (int i = 1; i <= 9; i++) {
            NamedCommands.registerCommand(
                    "TurretToHub" + i,
                    new ParallelDeadlineGroup(new WaitCommand(i),
                            new TurretToHub(turret, swerve, indexer, intake, swerveInput, led)));
        }

        // Configure button bindings
        configureBindings();

    }

    private void configureBindings() {

        // A Button: Zeroes the gyro when pressed, should only need if the camera
        // explodes
        driverController.a().onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

        // Left Trigger: Spins the intake wheel & spindexer foreward
        // Only if the right trigger isn't being held down
        driverController.leftTrigger().and(driverController.rightTrigger().negate()).whileTrue(runIntake);

        // Left Bumper: Spins the intake wheel & spindexer backward
        driverController.leftBumper().whileTrue(spindexerBackward.alongWith(runOutake));

        // Right Trigger: Spins the shooter wheel & spindexer while holding down
        driverController.rightTrigger().whileTrue(turretToHub);

        // X and Y: Intake in and out
        driverController.x().onTrue(Commands.runOnce(() -> intakeArm.setActive(), intakeArm));
        driverController.y().onTrue(Commands.runOnce(() -> intakeArm.setStowed(), intakeArm));

        buttonBoxController.a().onTrue(Commands.runOnce(() -> {
            if (Robot.noCameraMode != Robot.NoCameraMode.CLOSE_SHOT) {
                Robot.noCameraMode = Robot.NoCameraMode.CLOSE_SHOT;
            } else {
                Robot.noCameraMode = Robot.NoCameraMode.DISABLED;
            }
        }));

        buttonBoxController.b().onTrue(Commands.runOnce(() -> {
            if (Robot.noCameraMode != Robot.NoCameraMode.FAR_SHOT) {
                Robot.noCameraMode = Robot.NoCameraMode.FAR_SHOT;
            } else {
                Robot.noCameraMode = Robot.NoCameraMode.DISABLED;
            }
        }));

        buttonBoxController.x().onTrue(
                new SequentialCommandGroup(
                        new ZeroHood(turret),
                        new ZeroTurret(turret)));

        buttonBoxController.y().whileTrue(new OscillateIntake(intakeArm));

        buttonBoxController.leftBumper().onTrue(Commands.runOnce(() -> {turret.adjustLeft();}));
        buttonBoxController.rightBumper().onTrue(Commands.runOnce(() -> {turret.adjustRight();}));
        driverController.back().onTrue(Commands.runOnce(() -> {turret.adjustReset();}));
    }

    private int stateToButtonBox() {
        int[] buttonBoxLEDs = { 0, 0, 0, 0, 1, 1, 1, 1 };

        if (Robot.noCameraMode == Robot.NoCameraMode.CLOSE_SHOT) {
            buttonBoxLEDs[0] = globalTimer.get() % 0.5 < 0.25 ? 1 : 0;
        }

        if (Robot.noCameraMode == Robot.NoCameraMode.FAR_SHOT) {
            buttonBoxLEDs[1] = globalTimer.get() % 0.5 < 0.25 ? 1 : 0;
        }

        return buttonBoxLEDs[0] * 1 + buttonBoxLEDs[1] * 2 + buttonBoxLEDs[2] * 4 + buttonBoxLEDs[3] * 8
                + buttonBoxLEDs[4] * 16 + buttonBoxLEDs[5] * 32 + buttonBoxLEDs[6] * 64 + buttonBoxLEDs[7] * 128;
    }

    public void periodic() {
        swerve.getSwerveDrive().updateOdometry();
        vision.updateSwervePoseFromVision(swerve.getSwerveDrive());

        buttonBoxController.setRumble(RumbleType.kBothRumble, stateToButtonBox() / 255.0);

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
        // climber.setEnabled(climberEnabled);

        // SmartDashboard.putString("Robor Pos", swerve.getPose().toString());
        // SmartDashboard.putString("Turret Pose, ",
        // turret.getTurretPoseFromRobotPose(swerve.getPose()).toString());

        robotPosePublisher.set(swerve.getPose());
        turretPosePublisher.set(turret.getTurretPoseFromRobotPose(swerve.getPose()));

        // SmartDashboard.putData("PDH", pdh);

        // SmartDashboard.putData("Scheduled Commands", CommandScheduler.getInstance());

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

    public void scheduleAutonomous() {
        pathPlannerAutoCommand = autoChooser.getSelected();
        if (pathPlannerAutoCommand != null) {
            // run zeroing at the start of auto with a deadline of 1 second, then run the
            // path planner command after that
            hasRanCalibration = true; // Set this to true so that it doesn't run the calibration again in teleop
            CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                    new ZeroHood(turret),
                    new ZeroTurret(turret),
                    pathPlannerAutoCommand));
        } else {
            // if no path planner command is found, just run the zeroing commands
            hasRanCalibration = true; // Set this to true so that it doesn't run the calibration again in teleop
            CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                    new ZeroHood(turret),
                    new ZeroTurret(turret)));
        }
    }

    public void cancelAutonomous() {
        if (pathPlannerAutoCommand != null) {
            pathPlannerAutoCommand.cancel();
        }

        // Stop all subsystems just in case
        turret.stopHood();
        turret.stopShooter();
        indexer.stopIndexing();
        intake.stop();
    }

}