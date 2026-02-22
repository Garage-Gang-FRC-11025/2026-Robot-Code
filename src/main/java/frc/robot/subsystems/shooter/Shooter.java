package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  private static final LoggedTunableNumber wKP = new LoggedTunableNumber("shooter/Wheel/kP");
  private static final LoggedTunableNumber wKV = new LoggedTunableNumber("shooter/Wheel/kV");
  private static final LoggedTunableNumber hKP = new LoggedTunableNumber("shooter/Hood/kP");
  private static final LoggedTunableNumber hKD = new LoggedTunableNumber("shooter/Hood/kD");
  private static final LoggedTunableNumber rKP = new LoggedTunableNumber("shooter/rotation/kP");
  private static final LoggedTunableNumber rKD = new LoggedTunableNumber("shooter/rotation/kD");

  private static final LoggedTunableNumber wheelTargetAccelerationConfig =
      new LoggedTunableNumber("shooter/Wheel/Acceleration");
  private static final LoggedTunableNumber hoodMaxVelocityConfig =
      new LoggedTunableNumber("shooter/Hood/MaxVel");
  private static final LoggedTunableNumber hoodTargetAccelerationConfig =
      new LoggedTunableNumber("shooter/Hood/TargetAcceleration");
  private static final LoggedTunableNumber rotationMaxVelocityConfig =
      new LoggedTunableNumber("shooter/Rotation/MaxVel");
  private static final LoggedTunableNumber rotationTargetAccelerationConfig =
      new LoggedTunableNumber("shooter/Rotation/TargetAcceleration");

  static {
    if (Constants.currentMode == Mode.REAL) {
      wKP.initDefault(0.8);
      wKV.initDefault(0.15);
      wheelTargetAccelerationConfig.initDefault(300.0);

      hKP.initDefault(70.0);
      hKD.initDefault(1.6);

      hoodMaxVelocityConfig.initDefault(10);
      hoodTargetAccelerationConfig.initDefault(10);

      rKP.initDefault(70.0);
      rKD.initDefault(1.6);

      rotationMaxVelocityConfig.initDefault(10);
      rotationTargetAccelerationConfig.initDefault(10);
    } else {
      wKP.initDefault(0.0);
      wKV.initDefault(0.120);
      wheelTargetAccelerationConfig.initDefault(0.0);

      hKP.initDefault(2);
      hKD.initDefault(0);

      hoodMaxVelocityConfig.initDefault(40);
      hoodTargetAccelerationConfig.initDefault(80);

      rKP.initDefault(70.0);
      rKD.initDefault(1.6);

      rotationMaxVelocityConfig.initDefault(10);
      rotationTargetAccelerationConfig.initDefault(10);
    }
  }

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    configHood();
    configWheel();
    configRotation();
  }

  public void periodic() {
    Logger.processInputs("shooter", inputs);
    int hc = hashCode();
    if (wKV.hasChanged(hc) || wKP.hasChanged(hc) || wheelTargetAccelerationConfig.hasChanged(hc))
      configWheel();
    if (hKP.hasChanged(hc)
        || hKD.hasChanged(hc)
        || hoodMaxVelocityConfig.hasChanged(hc)
        || hoodTargetAccelerationConfig.hasChanged(hc)) configHood();
    if (rKP.hasChanged(hc)
        || rKD.hasChanged(hc)
        || rotationMaxVelocityConfig.hasChanged(hc)
        || rotationTargetAccelerationConfig.hasChanged(hc)) configRotation();
    shooterIO.updateInputs(inputs);
  }

  private void configHood() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = hoodTargetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = hoodMaxVelocityConfig.get();
    shooterIO.configHood(hKP.get(), hKD.get(), mmConfigs);
  }

  private void configRotation() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = rotationTargetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = rotationMaxVelocityConfig.get();
    shooterIO.configRotation(rKP.get(), rKD.get(), mmConfigs);
  }

  private void configWheel() {
    shooterIO.configWheel(wKV.get(), wKP.get(), wheelTargetAccelerationConfig.get());
  }

  public void setWheelVoltage(double volts) {
    shooterIO.setWheelVoltage(volts);
  }

  public void setrotationVoltage(double volts) {
    shooterIO.setRotationVoltage(volts);
  }

  public void setHoodVoltage(double volts) {
    shooterIO.setHoodVoltage(volts);
  }

  public void setWheelVel(AngularVelocity vel) {
    shooterIO.setWheelVel(vel);
  }

  public void sethoodPos(Rotation2d pos) {
    shooterIO.setHoodPos(pos);
  }

  public void setRotationPos(Rotation2d pos) {
    shooterIO.setRotationPos(pos);
  }
}
