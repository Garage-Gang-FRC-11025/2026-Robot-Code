package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.RotationConstants;
import frc.robot.Constants.ShooterConstants.WheelConstants;
import frc.robot.util.TalonFXArmSim;
import frc.robot.util.TalonFXSim;

public class ShooterIOSim implements ShooterIO {

  private TalonFXSim wheelMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), WheelConstants.WHEEL_GEARING, WheelConstants.WHEEL_MOI);

  private TalonFXArmSim HoodSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              frc.robot.Constants.ShooterConstants.HoodConstants.GEAR_RATIO,
              HoodConstants.HOOD_MOI,
              HoodConstants.HOOD_LENGTH.in(Units.Meters),
              HoodConstants.MIN_HOOD_ANGLE.getRadians(),
              HoodConstants.MAX_HOOD_ANGLE.getRadians(),
              false,
              0));

  private TalonFXArmSim RotationSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(1),
              frc.robot.Constants.ShooterConstants.HoodConstants.GEAR_RATIO,
              RotationConstants.ROTATION_MOI,
              0.5,
              RotationConstants.MIN_ROTATION_ANGLE.getRadians(),
              RotationConstants.MAX_ROTATION_ANGLE.getRadians(),
              false,
              0));

  private VoltageOut wheelOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage wheelClosedLoopControl = new VelocityVoltage(0);

  private VoltageOut hoodOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage hoodClosedLoopControl = new MotionMagicVoltage(0);

  private VoltageOut rotationOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage rotationClosedLoopControl = new MotionMagicVoltage(0);

  public ShooterIOSim() {}

  @Override
  public void updateInputs(shooterInputs inputs) {
    wheelMotor.update(Constants.kDefaultPeriod);
    inputs.wheelAppliedOutput = wheelMotor.getVoltage();
    inputs.wheelsVelocityRPM = wheelMotor.getVelocity().in(Units.RPM);

    HoodSim.update(Constants.kDefaultPeriod);
    inputs.hoodPosition = new Rotation2d(HoodSim.getPosition());
    inputs.hoodAppliedOutput = HoodSim.getVoltage().in(Units.Volts);
    inputs.hoodVelocity = HoodSim.getVelocity().in(Units.DegreesPerSecond);
    RotationSim.update(Constants.kDefaultPeriod);
    inputs.rotationPosition = new Rotation2d(RotationSim.getPosition());
    inputs.rotationAppliedOutput = RotationSim.getVoltage().in(Units.Volts);
    inputs.rotationVelocity = RotationSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setWheelVoltage(double volts) {
    wheelMotor.setControl(wheelOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setWheelVel(AngularVelocity revPerMin) {
    wheelMotor.setControl(wheelClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void configWheel(double kV, double kP, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;

    wheelMotor.setConfig(config);
  }

  @Override
  public void setHoodVoltage(double volts) {
    HoodSim.setControl(hoodOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setHoodPos(Rotation2d angle) {
    HoodSim.setControl(hoodClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void configHood(double kP, double kD, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    HoodSim.setConfig(config);
  }

  @Override
  public void setRotationVoltage(double volts) {
    RotationSim.setControl(rotationOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRotationPos(Rotation2d angle) {
    RotationSim.setControl(rotationClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void configRotation(double kP, double kD, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    RotationSim.setConfig(config);
  }
}
