// package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.WheelConstants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.constants.ShooterConstants.RotationConstants;

// public class ShooterIOReal implements ShooterIO {

  private final TalonFX wheelMotor = new TalonFX(Constants.CanIDs.SHOOTER_WHEEL_CAN_ID);
  private final TalonFX hoodMotor = new TalonFX(Constants.CanIDs.SHOOTER_HOOD_CAN_ID);
  private final TalonFX rotationMotor = new TalonFX(Constants.CanIDs.SHOOTER_ROTATION_CAN_ID);


  private final StatusSignal<Current> wheelCurrent;
  private final StatusSignal<Temperature> wheelDeviceTemp;
  private final StatusSignal<Voltage> wheelAppliedVoltage;
  private final StatusSignal<AngularVelocity> wheelVelocity;
  private final StatusSignal<Current> rotationCurrent;
  private final StatusSignal<Temperature> rotationDeviceTemp;
  private final StatusSignal<Voltage> rotationAppliedVoltage;
  private final StatusSignal<AngularVelocity> rotationVelocity;
  private final StatusSignal<Temperature> hoodDeviceTemp;
  private final StatusSignal<Voltage> hoodAppliedVoltage;
  private final StatusSignal<Angle> hoodAngle;
  private final StatusSignal<Current> hoodCurrent;

  private final VoltageOut wheelOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut hoodOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut rotationOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage wheelClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage rotationClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage hoodClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public ShooterIOReal() {
    // Motor config
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs wheelCurrentLimitConfig = new CurrentLimitsConfigs();

    wheelCurrentLimitConfig.SupplyCurrentLimit = WheelConstants.SUPPLY_CURRENT_LIMIT;
    wheelCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    wheelCurrentLimitConfig.StatorCurrentLimit = WheelConstants.STATOR_CURRENT_LIMIT;
    wheelCurrentLimitConfig.StatorCurrentLimitEnable = true;

    wheelConfig.CurrentLimits = wheelCurrentLimitConfig;

    wheelConfig.Feedback.SensorToMechanismRatio = wheelConstants.WHEEL_GEARING;
    wheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    wheelConfig.Voltage.SupplyVoltageTimeConstant = WheelConstants.SUPPLY_VOLTAGE_TIME;

    wheelMotor.getConfigurator().apply(wheelConfig);

    // Status signals

    wheelCurrent = wheelMotor.getStatorCurrent();
    wheelDeviceTemp = wheelMotor.getDeviceTemp();
    wheelAppliedVoltage = wheelMotor.getMotorVoltage();
    wheelVelocity = wheelMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, wheelAppliedVoltage, wheelCurrent, wheelVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, wheelDeviceTemp);

    wheelMotor.optimizeBusUtilization();

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.MAX_HOOD_ANGLE.getRotations();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.MIN_HOOD_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.Voltage.SupplyVoltageTimeConstant = HoodConstants.SUPPLY_VOLTAGE_TIME;

    hoodMotor.getConfigurator().apply(hoodConfig);

    // Status signals

    hoodAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodAngle = hoodMotor.getPosition();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodDeviceTemp = hoodMotor.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, hoodAppliedVoltage, hoodAngle);
    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, hoodDeviceTemp);

    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(
        wheelAppliedVoltage,
        wheelCurrent,
        wheelDeviceTemp,
        wheelVelocity,
        hoodAngle,
        hoodCurrent,
        hoodAppliedVoltage,
        hoodDeviceTemp
        rotationAngle,
        rotationCurrent,
        rotationAppliedVoltage,
        rotationDeviceTemp        
        
        );

    inputs.wheelCurrentAmps = wheelCurrent.getValue().in(Units.Amps);
    inputs.wheelTempCelsius = wheelDeviceTemp.getValue().in(Units.Celsius);
    inputs.wheelAppliedOutput = wheelAppliedVoltage.getValue().in(Units.Volts);
    inputs.wheelVelocityRPM = wheelVelocity.getValue().in(Units.RPM);
  }

  @Override
  public void setWheelVoltage(double volts) {
    wheelMotor.setControl(wheelOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setWheelVel(AngularVelocity vel) {
    wheelMotor.setControl(wheelClosedLoopControl.withVelocity(vel.in(Units.RotationsPerSecond)));
  }

  @Override
  public void configWheel(double kV, double kP, double maxAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var rollerConfig = wheelMotor.getConfigurator();

    wheelConfig.refresh(pidConfig);
    wheelConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = maxAcceleration;

    wheelConfig.apply(pidConfig);
    wheelConfig.apply(mmConfig);
  }

  @Override
  public void setExtenderPos(Rotation2d pos) {
    hoodClosedLoopControl.withPosition(pos.getRotations());
    hoodMotor.setControl(hoodClosedLoopControl);
  }

  @Override
  public void configExtender(double kP, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    hoodMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    hoodMotor.getConfigurator().apply(slot0Configs);
    hoodMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setControl(hoodOpenLoopControl.withOutput(volts));
  }

  @Override
  public boolean setHoodNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = hoodMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    hoodMotor.getConfigurator().apply(config);
    return true;
  }
}