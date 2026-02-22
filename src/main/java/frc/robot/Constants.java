// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

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

  public class IntakeConstants {
    public class ExtenderConstants {

      public static final double GEAR_RATIO = 75; // will need to be changed ALOT
      public static final Rotation2d MAX_EXTENDER_ANGLE = Rotation2d.fromDegrees(130);
      public static final Rotation2d MIN_EXTENDER_ANGLE = Rotation2d.fromDegrees(0);
      public static final double SUPPLY_CURRENT_LIMIT = 0;
      public static final double STATOR_CURRENT_LIMIT = 0;
      public static final double SUPPLY_VOLTAGE_TIME = 0;
      public static final double EXTENDER_MOI = 0.1;
      public static final Distance EXTENDER_LENGTH = Units.Inches.of(12.5);
    }

    public class RollerConstants {

      public static final double SUPPLY_CURRENT_LIMIT = 0;
      public static final double STATOR_CURRENT_LIMIT = 0;
      public static final double ROLLER_GEARING = 3;
      public static final double SUPPLY_VOLTAGE_TIME = 0;
      public static final double ROLLER_MOI = 0.1;
    }
  }

  public class ElevatorConstants {

    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double STATOR_CURRENT_LIMIT = 30;
    public static final double SUPPLY_VOLTAGE_TIME = 0;
    public static final double ELEVATOR_GEARING = 1;
    public static final double ELEVATOR_MOI = 0.1;
  }

  public class CanIDs {
    public static final int FRONT_LEFT_WHEEL_CAN_ID = 1;
    public static final int FRONT_LEFT_ANGLE_CAN_ID = 2;
    public static final int FRONT_RIGHT_WHEEL_CAN_ID = 3;
    public static final int FRONT_RIGHT_ANGLE_CAN_ID = 4;
    public static final int BACK_LEFT_WHEEL_CAN_ID = 5;
    public static final int BACK_LEFT_ANGLE_CAN_ID = 6;
    public static final int BACK_RIGHT_WHEEL_CAN_ID = 7;
    public static final int BACK_RIGHT_ANGLE_CAN_ID = 8;
    public static final int INTAKE_EXTENDER_CAN_ID = 9;
    public static final int INTAKE_ROLLER_CAN_ID = 10;
    public static final int ELEVATOR_ROLLER_CAN_ID = 11;
    public static final int ELEVATOR_CAN_ID = 12;
    public static final int SHOOTER_WHEEL_CAN_ID = 13;
    public static final int SHOOTER_HOOD_CAN_ID = 14;
    public static final int SHOOTER_ROTATION_CAN_ID = 15;
  }

  public final class ShooterConstants {

    public final class HoodConstants {
      public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d MAX_HOOD_ANGLE = Rotation2d.fromDegrees(90);
      public static final double HOOD_MOI = 0.1;
      public static final double STATOR_CURRENT_LIMIT = 20;
      public static final double SUPPLY_CURRENT_LIMIT = 20;
      public static final double MAX_VELOCITY = 10;
      public static final double TARGET_ACCELERATION = 10;
      public static final double GEAR_RATIO = 0.017647;
      public static final double SUPPLY_VOLTAGE_TIME = 0.5;
      public static final double MAX_VOLTAGE = 12.0;
      public static final double kDefaultPeriod = 0.02;
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
