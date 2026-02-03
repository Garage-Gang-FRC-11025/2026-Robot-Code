// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber turretMaxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");
  private static final LoggedTunableNumber turretMaxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration", 9999999);
  private static final LoggedTunableNumber turretkP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber turretkD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber turretkA = new LoggedTunableNumber("Turret/kA");

  static {
    if (Constants.currentMode == Mode.REAL) {

      turretMaxVelocity.initDefault(0.5);
      turretkP.initDefault(500.0);
      turretkD.initDefault(0.0);
      turretkA.initDefault(0.0);
    } else {
      turretMaxVelocity.initDefault(16.0);
      turretkP.initDefault(3500.0);
      turretkD.initDefault(150.0);
      turretkA.initDefault(0.0);
    }
  }
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
