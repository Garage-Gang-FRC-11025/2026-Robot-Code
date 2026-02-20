// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.Units;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

      public class CanIDs {
      public static final int SHOOTER_WHEEL_CAN_ID = 6;
      public static final int SHOOTER_HOOD_CAN_ID = 7;
      public static final int SHOOTER_ROTATION_CAN_ID = 8;
    }

  public final class ShooterConstants {
    public static final double GEAR_RATIO = 1.0;

    public final class HoodConstants {
      public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d MAX_HOOD_ANGLE = Rotation2d.fromDegrees(90);
      public static final double HOOD_MOI = 0.1;
      public static final double STATOR_CURRENT_LIMIT = 20;
      public static final double SUPPLY_CURRENT_LIMIT = 20;
      public static final double MAX_VELOCITY = 10;
      public static final double TARGET_ACCELERATION = 10;
      public static final double GEAR_RATIO = 1.0;
      public static final double SUPPLY_VOLTAGE_TIME = 0.5;
      public static final Distance HOOD_LENGTH = Units.Inches.of(1);
    }

    public final class RotationConstants {
      public static final Rotation2d MIN_ROTATION_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d MAX_ROTATION_ANGLE = Rotation2d.fromDegrees(180);
      public static final double ROTATION_MOI = 0.1;
      public static final double STATOR_CURRENT_LIMIT = 20;
      public static final double SUPPLY_CURRENT_LIMIT = 20;
      public static final double MAX_VELOCITY = 10;
      public static final double TARGET_ACCELERATION = 10;
      public static final double GEAR_RATIO = 6.6667;
      public static final double SUPPLY_VOLTAGE_TIME = 0.5;
    }

        public final class WheelConstants {
      public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches
      public static final double WHEEL_GEARING = 1.0;
      public static final double WHEEL_MOI = 0.1;
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double SUPPLY_VOLTAGE_TIME = 0.5;
        }
  }

  public static final double MAX_VOLTAGE = 12.0;
  public static final double kDefaultPeriod = 0.02;

}
