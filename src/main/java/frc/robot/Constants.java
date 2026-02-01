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
  public class IntakeConstants {

    public class RollerConstants{

        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double STATOR_CURRENT_LIMIT = 0;
        public static final double ROLLER_GEARING = 0;
        public static final double SUPPLY_VOLTAGE_TIME = 0;



    }
    public class ExtenderConstants{

        public static final double GEAR_RATIO = 0;
        public static final Rotation2d MAX_EXTENDER_ANGLE = null;
        public static final Rotation2d MIN_EXTENDER_ANGLE = null;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double STATOR_CURRENT_LIMIT = 0;
        public static final double SUPPLY_VOLTAGE_TIME = 0;
        public static final double PULLEY_DIAMETER_INCHES = 0;
        public static final double INCHES_TO_MOTOR_ROT = GEAR_RATIO / (Math.PI * PULLEY_DIAMETER_INCHES);

    }

  }
    public class CanIDs {
      public static final int INTAKE_ROLLER_CAN_ID = 9;
      public static final int INTAKE_EXTENDER_CAN_ID = 10;

  }
}
