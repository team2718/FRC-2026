// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  // Velocity filtering/estimation options
  private static final double fieldVelocityFilterGain = 0.2;
  private static final double velocityObserverFeedbackGain = 0.15;
  private static final double velocityObserverFeedForwardGain = 0.85;
  private static final boolean useVelocityObserver = false; // Set to true to use observer instead of simple low-pass

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;

  private boolean enabled = true;
  private ChassisSpeeds filteredFieldVelocity = new ChassisSpeeds();
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private boolean hasFilteredFieldVelocity = false;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */

  public SwerveSubsystem(File directory) {

    if (Constants.REDUCED_TELEMETRY) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED.in(MetersPerSecond));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(true);

    setupPathPlanner();

    // TODO: test this later (see
    // https://docs.yagsl.com/overview/our-features/angular-velocity-compensation)
    // swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

    // Stop the default odometry thread since we will be handling odometry manually
    // when using vision.
    swerveDrive.stopOdometryThread();
  }

  public SwerveSubsystem() {
    this(new File(Filesystem.getDeployDirectory(), "swerve"));
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (!enabled) {
              speedsRobotRelative = new ChassisSpeeds(0, 0, 0);
              moduleFeedForwards = DriveFeedforwards.zeros(swerveDrive.getModules().length);
            }

            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    if (!enabled) {
      translation = new Translation2d(0, 0);
      rotation = 0;
    }
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    if (!enabled) {
      velocity = new ChassisSpeeds(0, 0, 0);
    }
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get().times(enabled ? 1 : 0));
    });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command drive(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.drive(velocity.get().times(enabled ? 1 : 0));
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    if (!enabled) {
      velocity = new ChassisSpeeds(0, 0, 0);
    }
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
    resetFilteredFieldVelocity();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
    resetFilteredFieldVelocity();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED.in(MetersPerSecond));
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED.in(MetersPerSecond));
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getFilteredFieldVelocity() {
    if (!hasFilteredFieldVelocity) {
      resetFilteredFieldVelocity();
    }

    return filteredFieldVelocity;
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    swerveDrive.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }

  public void updateOdometry() {
    swerveDrive.updateOdometry();
    updateFilteredFieldVelocity();
  }

  public void setLastCommandedVelocity(ChassisSpeeds velocity) {
    this.lastCommandedVelocity = velocity;
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveInputStream getAngularVelocityRobotRelativeInputStream(CommandXboxController driverController) {
    return SwerveInputStream.of(swerveDrive,
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1)
        .withControllerRotationAxis(() -> driverController.getRightX() * -1)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(OperatorConstants.SPEED_MULTIPLIER)
        .scaleRotation(OperatorConstants.ROTATION_MULTIPLIER)
        .allianceRelativeControl(false)
        .robotRelative(false);
  }

  public SwerveInputStream getAngularVelocityFieldRelativeInputStream(CommandXboxController driverController) {
    return getAngularVelocityRobotRelativeInputStream(driverController)
        .allianceRelativeControl(true);
  }

  public SwerveInputStream getDirectAngleFieldRelativeInputStream(CommandXboxController driverController) {
    return getAngularVelocityRobotRelativeInputStream(driverController)
        .withControllerHeadingAxis(driverController::getRightX, driverController::getRightY)
        .headingWhile(true)
        .allianceRelativeControl(true);
  }

  public static ChassisSpeeds applyAccelLimit(ChassisSpeeds current, ChassisSpeeds target, double accelLimit) {
    double dvx = target.vxMetersPerSecond - current.vxMetersPerSecond;
    double dvy = target.vyMetersPerSecond - current.vyMetersPerSecond;
    double dvMag = Math.hypot(dvx, dvy);

    if (dvMag > accelLimit) {
      double scale = accelLimit / dvMag;
      return new ChassisSpeeds(
          current.vxMetersPerSecond + (dvx * scale),
          current.vyMetersPerSecond + (dvy * scale),
          target.omegaRadiansPerSecond);
    }
    return target;
  }

  public static ChassisSpeeds applyVelocityLimit(ChassisSpeeds target, double velocityLimit) {
    double speed = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond);

    if (speed > velocityLimit) {
      double scale = velocityLimit / speed;
      return new ChassisSpeeds(
          target.vxMetersPerSecond * scale,
          target.vyMetersPerSecond * scale,
          target.omegaRadiansPerSecond);
    }
    return target;
  }

  public static ChassisSpeeds applyAccelLimit(ChassisSpeeds current, ChassisSpeeds target, double accelLimit,
      double angularAccelLimit) {
    // Translation Acceleration
    double dvx = target.vxMetersPerSecond - current.vxMetersPerSecond;
    double dvy = target.vyMetersPerSecond - current.vyMetersPerSecond;
    double dvMag = Math.hypot(dvx, dvy);

    double nextVx = target.vxMetersPerSecond;
    double nextVy = target.vyMetersPerSecond;

    if (dvMag > accelLimit) {
      double scale = accelLimit / dvMag;
      nextVx = current.vxMetersPerSecond + (dvx * scale);
      nextVy = current.vyMetersPerSecond + (dvy * scale);
    }

    // Rotation Acceleration
    double dOmega = target.omegaRadiansPerSecond - current.omegaRadiansPerSecond;
    double nextOmega = target.omegaRadiansPerSecond;

    if (Math.abs(dOmega) > angularAccelLimit) {
      nextOmega = current.omegaRadiansPerSecond + Math.copySign(angularAccelLimit, dOmega);
    }

    return new ChassisSpeeds(nextVx, nextVy, nextOmega);
  }

  public static ChassisSpeeds applyVelocityLimit(ChassisSpeeds target, double velocityLimit, double rotationLimit) {
    // Translation Velocity Limit
    double speed = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond);

    double nextVx = target.vxMetersPerSecond;
    double nextVy = target.vyMetersPerSecond;

    if (speed > velocityLimit) {
      double scale = velocityLimit / speed;
      nextVx = target.vxMetersPerSecond * scale;
      nextVy = target.vyMetersPerSecond * scale;
    }

    // Rotation Velocity Limit
    double nextOmega = target.omegaRadiansPerSecond;
    if (Math.abs(nextOmega) > rotationLimit) {
      nextOmega = Math.copySign(rotationLimit, nextOmega);
    }

    return new ChassisSpeeds(nextVx, nextVy, nextOmega);
  }

  private void resetFilteredFieldVelocity() {
    filteredFieldVelocity = swerveDrive.getFieldVelocity();
    lastCommandedVelocity = swerveDrive.getFieldVelocity();
    hasFilteredFieldVelocity = true;
  }

  private void updateFilteredFieldVelocity() {
    ChassisSpeeds rawFieldVelocity = swerveDrive.getFieldVelocity();

    if (!hasFilteredFieldVelocity) {
      filteredFieldVelocity = rawFieldVelocity;
      hasFilteredFieldVelocity = true;
      return;
    }

    if (useVelocityObserver) {
      filteredFieldVelocity = estimateVelocityWithObserver(rawFieldVelocity);
    } else {
      filteredFieldVelocity = new ChassisSpeeds(
          blend(filteredFieldVelocity.vxMetersPerSecond, rawFieldVelocity.vxMetersPerSecond),
          blend(filteredFieldVelocity.vyMetersPerSecond, rawFieldVelocity.vyMetersPerSecond),
          blend(filteredFieldVelocity.omegaRadiansPerSecond, rawFieldVelocity.omegaRadiansPerSecond));
    }
  }

  private ChassisSpeeds estimateVelocityWithObserver(ChassisSpeeds measuredVelocity) {
    return new ChassisSpeeds(
        observerBlend(filteredFieldVelocity.vxMetersPerSecond, lastCommandedVelocity.vxMetersPerSecond, measuredVelocity.vxMetersPerSecond),
        observerBlend(filteredFieldVelocity.vyMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond, measuredVelocity.vyMetersPerSecond),
        observerBlend(filteredFieldVelocity.omegaRadiansPerSecond, lastCommandedVelocity.omegaRadiansPerSecond, measuredVelocity.omegaRadiansPerSecond));
  }

  private double observerBlend(double estimatedVelocity, double commandedVelocity, double measuredVelocity) {
    return (velocityObserverFeedForwardGain * commandedVelocity) + (velocityObserverFeedbackGain * measuredVelocity);
  }

  private double blend(double previousValue, double currentValue) {
    return previousValue + (fieldVelocityFilterGain * (currentValue - previousValue));
  }
}
