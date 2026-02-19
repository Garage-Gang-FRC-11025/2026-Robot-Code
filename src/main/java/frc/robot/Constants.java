// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public final class ShooterConstants {
    public static final double GEAR_RATIO = 1.0;

    public final class TurretConstants {
      public static final Rotation2d HORIZONTAL_MIN_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d HORIZONTAL_MAX_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d VERTICAL_MIN_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d VERTICAL_MAX_ANGLE = Rotation2d.fromDegrees(0);
    }

    public class CanIDs {
      public static final int HORIZONTAL_ANGLE_MOTOR_CAN_ID = 6;
      public static final int VERTICAL_ANGLE_MOTOR_CAN_ID = 7;
      public static final int SHOOTER_MOTOR_CAN_ID = 8;
    }
  }
}
