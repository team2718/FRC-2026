// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Constants, copied from the basketball robot's code
  public static final boolean COMPETITION_MODE = false;
  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(14.0);

  public static class OperatorConstants {
    // Operator constants, borrowed from the basketball robot's code
    public static final int kDriverControllerPort = 0;
    public static final double SPEED_MULTIPLIER = 0.25;
    public static final double ROTATION_MULTIPLIER = 0.5;
    public static final double DEADBAND = 0.1;
  }

  public static final class TurretConstants
  {
    //placeholder ids
    public static final int turretspinnerID = 10;
    public static final int turrethoodID = 42;
    public static final int turretshooterLeftID = 21;
    public static final int turretshooterRightID = 20;
  }

  public static final class ClimberConstants {
    public static final int climbMotorID = 51;
    public static final double BAR1_ELEVATION = Units.inchesToMeters(27.0);
    public static final double BAR2_ELEVATION = Units.inchesToMeters(45.0);
    public static final double BAR3_ELEVATION = Units.inchesToMeters(63.0);
    public static final double L1_ELEVATION = Units.inchesToMeters(2.0);
    public static final double L2_ELEVATION = BAR1_ELEVATION + Units.inchesToMeters(2);
    public static final double L3_ELEVATION = BAR2_ELEVATION + Units.inchesToMeters(2);
    public static final double EXTEND_P = 0.03;
    public static final double RETRACT_P = 0.06;
    public static final double HOOK_BASE_ELEVATION = Units.inchesToMeters(19); // Estimated value
    public static final double CLIMB_TOLERANCE = 0.1;
  }

    public static final class IndexerConstants
  {
    //placeolder ids
    public static final int indexermotor1ID = 66;
    public static final int indexermotor2ID = 68;
  }

    public static final class IntakeConstants
  {
    //placeholder ids
    public static final int intakemotorID = 7;
  }
  /*
  public static class OI {

    //xbox button ids
    public static final int kdriverControllerButton1 = 1;
    public static final int kdriverControllerButton2 = 2;
    public static final int kdriverControllerButton3 = 3;
    public static final int kdriverControllerButton4 = 4;
    public static final int kdriverControllerButton5 = 5;
    public static final int kdriverControllerButton6 = 6;
    public static final int kdriverControllerButton7 = 7;
    public static final int kdriverControllerButton8 = 8;
  }
  */
  public static final class LEDS{
    //these are random numbers... they haven't been tested yet
    public static final int PWMPort = 0;
    public static final int Length = 12;
  }
}
