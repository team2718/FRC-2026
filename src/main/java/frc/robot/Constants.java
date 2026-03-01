// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Set to true if performance or space become an issue. This will disable all
  // telemetry except for critical errors and warnings.
  public static final boolean REDUCED_TELEMETRY = false;

  public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(14.0);

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static class OperatorConstants {
    public static final double SPEED_MULTIPLIER = 0.4;
    public static final double ROTATION_MULTIPLIER = 0.5;
    public static final double DEADBAND = 0.1;
  }

  public static final class TurretConstants {
    // public static final int turretSpinnerID = 17;
    public static final int HOOD_MOTOR_ID = 15;
    public static final int LEFT_SHOOTER_MOTOR_ID = 14;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 13;
  }

  public static final class ClimberConstants {
    public static final int MOTOR_ID = 10;
    public static final int CURRENT_LIMIT = 40;

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

  public static final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 9;
    public static final int PORTAL_MOTOR_ID = 16;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 12;
    public static final int SLAPDOWN_MOTOR_ID = 11;
  }

  public static final class LEDS {
    public static final int PWM_PORT = 0;
    public static final int NUM_LEDS = 12;
  }

  public static final class RebuiltMatchPeriods {
    // Match times via '6.4 MATCH Periods'
    public static final double AUTONOMOUS_DURATION = 20.0;
    public static final double TRANSITION_SHIFT_DURATION = 10.0;
    public static final double SHIFT_DURATION = 25.0;
    public static final double END_GAME_DURATION = 30.0;

    public static enum AutoWinner {
      RED("Red"),
      BLUE("Blue"),
      UNKNOWN("Unknown");

      private final String displayName;

      private AutoWinner(String displayName) {
        this.displayName = displayName;
      }

      @Override
      public String toString() {
        return displayName;
      }

      public static AutoWinner fromGameData(String gameData) {
        if (gameData == null || gameData.isEmpty()) {
          return UNKNOWN;
        }

        switch (gameData.charAt(0)) {
          case 'R':
            return RED;
          case 'B':
            return BLUE;
          default:
            return UNKNOWN;
        }
      }

      public AutoWinner opposite() {
        switch (this) {
          case RED:
            return BLUE;
          case BLUE:
            return RED;
          default:
            return UNKNOWN;
        }
      }
    }

    public static enum MatchPeriod {
      AUTO("Auto"),
      TRANSITION_SHIFT("Transition Shift"),
      SHIFT_1("Shift 1"),
      SHIFT_2("Shift 2"),
      SHIFT_3("Shift 3"),
      SHIFT_4("Shift 4"),
      END_GAME("End Game");

      private final String displayName;

      private MatchPeriod(String displayName) {
        this.displayName = displayName;
      }

      @Override
      public String toString() {
        return displayName;
      }
    }

    public static MatchPeriod getTeleopPeriodFromTime(double time) {
      if (time < TRANSITION_SHIFT_DURATION) {
        return MatchPeriod.TRANSITION_SHIFT;
      } else if (time < TRANSITION_SHIFT_DURATION + SHIFT_DURATION) {
        return MatchPeriod.SHIFT_1;
      } else if (time < TRANSITION_SHIFT_DURATION + 2 * SHIFT_DURATION) {
        return MatchPeriod.SHIFT_2;
      } else if (time < TRANSITION_SHIFT_DURATION + 3 * SHIFT_DURATION) {
        return MatchPeriod.SHIFT_3;
      } else if (time < TRANSITION_SHIFT_DURATION + 4 * SHIFT_DURATION) {
        return MatchPeriod.SHIFT_4;
      } else {
        return MatchPeriod.END_GAME;
      }
    }

    public static double getTimeLeftInCurrentPeriod(double time) {
      if (time < TRANSITION_SHIFT_DURATION) {
        return TRANSITION_SHIFT_DURATION - time;
      } else if (time < TRANSITION_SHIFT_DURATION + SHIFT_DURATION) {
        return TRANSITION_SHIFT_DURATION + SHIFT_DURATION - time;
      } else if (time < TRANSITION_SHIFT_DURATION + 2 * SHIFT_DURATION) {
        return TRANSITION_SHIFT_DURATION + 2 * SHIFT_DURATION - time;
      } else if (time < TRANSITION_SHIFT_DURATION + 3 * SHIFT_DURATION) {
        return TRANSITION_SHIFT_DURATION + 3 * SHIFT_DURATION - time;
      } else if (time < TRANSITION_SHIFT_DURATION + 4 * SHIFT_DURATION) {
        return TRANSITION_SHIFT_DURATION + 4 * SHIFT_DURATION - time;
      } else {
        return 2 * 60 + 20 - time;
      }
    }
  }
}
