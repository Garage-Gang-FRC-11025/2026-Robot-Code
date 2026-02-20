package frc.robot.subsystems.shooter;

// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.AngularVelocity;
// import frc.robot.Constants;
// import frc.robot.Constants.ShooterConstants;
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
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.RotationConstants;
import frc.robot.Constants.ShooterConstants.WheelConstants;

public class ShooterIOReal implements ShooterIO {

//   private final SparkFlex leadMotor =
//       new SparkFlex(Constants.ShooterConstants.CanIDs.SHOOTER_MOTOR_CAN_ID,
// MotorType.kBrushless);
//   private SparkFlexConfig config = ShooterConstants.MOTOR_CONFIG();

//   private final RelativeEncoder encoder = leadMotor.getEncoder();

//   private final SparkFlexConfig followConfig = new SparkFlexConfig();

//   // private final VL6180 timeOfFlight = new VL6180(Port.kOnboard);

//   @Override
//   public void updateInputs(ShooterInputs inputs) {
//     inputs.velocityRPM = encoder.getVelocity();
//     inputs.appliedOutput = leadMotor.getAppliedOutput();
//     inputs.leadCurrentAmps = leadMotor.getOutputCurrent();
//     inputs.leadTempCelsius = leadMotor.getMotorTemperature();

//     // inputs.tofDistanceInches = timeOfFlight.getDistance().in(Units.Inches);
//   }

//   @Override
//   public void setVoltage(double volts) {
//     leadMotor.setVoltage(volts);
//   }

//   @Override
//   public void setVel(AngularVelocity angle) {
//     leadMotor
//         .getClosedLoopController()
//         .setReference(
//             angle.in(Units.RPM) / ShooterConstants.GEAR_RATIO,
//             ControlType.kMAXMotionVelocityControl);
//   }

//   @Override
//   public void configMotor(double kV, double kP, double maxAcceleration) {
//     config.closedLoop.pidf(kP, 0, 0, kV);
//     config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
//     leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   @Override
//   public boolean setIdleMode(IdleMode value) {
//     config.idleMode(value);
//     followConfig.idleMode(value);
//     return leadMotor.configure(
//                 config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
//             == REVLibError.kOk
//         && followMotor.configure(
//                 followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
//             == REVLibError.kOk;
//   }
// }
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
  private final StatusSignal<Angle> rotationAngle;
  private final StatusSignal<Temperature> hoodDeviceTemp;
  private final StatusSignal<Voltage> hoodAppliedVoltage;
  private final StatusSignal<Angle> hoodAngle;
  private final StatusSignal<Current> hoodCurrent;

  private final VoltageOut wheelOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut hoodOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut rotationOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage wheelClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage rotationClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage hoodClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public ShooterIOReal() {
    // Motor config
    TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs wheelCurrentLimitConfig = new CurrentLimitsConfigs();

    wheelCurrentLimitConfig.SupplyCurrentLimit = WheelConstants.SUPPLY_CURRENT_LIMIT;
    wheelCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    wheelCurrentLimitConfig.StatorCurrentLimit = WheelConstants.STATOR_CURRENT_LIMIT;
    wheelCurrentLimitConfig.StatorCurrentLimitEnable = true;
    wheelConfig.CurrentLimits = wheelCurrentLimitConfig;

    wheelConfig.Feedback.SensorToMechanismRatio = WheelConstants.WHEEL_GEARING;
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

    BaseStatusSignal.setUpdateFrequencyForAll(50, wheelAppliedVoltage, wheelCurrent, wheelVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, wheelDeviceTemp);

    wheelMotor.optimizeBusUtilization();

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();

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

    rotationConfig.CurrentLimits.SupplyCurrentLimit = RotationConstants.SUPPLY_CURRENT_LIMIT;
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.StatorCurrentLimit = RotationConstants.STATOR_CURRENT_LIMIT;
    rotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        RotationConstants.MAX_ROTATION_ANGLE.getRotations();
    rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        RotationConstants.MIN_ROTATION_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    rotationConfig.Feedback.SensorToMechanismRatio = RotationConstants.GEAR_RATIO;
    rotationConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    rotationConfig.Voltage.SupplyVoltageTimeConstant = HoodConstants.SUPPLY_VOLTAGE_TIME;

    rotationMotor.getConfigurator().apply(hoodConfig);

    // Status signals

    hoodAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodAngle = hoodMotor.getPosition();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodDeviceTemp = hoodMotor.getDeviceTemp();

    rotationAppliedVoltage = hoodMotor.getMotorVoltage();
    rotationAngle = hoodMotor.getPosition();
    rotationCurrent = hoodMotor.getStatorCurrent();
    rotationDeviceTemp = hoodMotor.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, hoodAppliedVoltage, hoodAngle);
    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, hoodDeviceTemp);

    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(shooterInputs inputs) {
    BaseStatusSignal.refreshAll(
        wheelAppliedVoltage,
        wheelCurrent,
        wheelDeviceTemp,
        wheelVelocity,
        hoodAngle,
        hoodCurrent,
        hoodAppliedVoltage,
        hoodDeviceTemp,
        rotationAngle,
        rotationCurrent,
        rotationAppliedVoltage,
        rotationDeviceTemp);

    inputs.wheelCurrentAmps = wheelCurrent.getValue().in(Units.Amps);
    inputs.wheelTempCelsius = wheelDeviceTemp.getValue().in(Units.Celsius);
    inputs.wheelAppliedOutput = wheelAppliedVoltage.getValue().in(Units.Volts);
    inputs.wheelsVelocityRPM = wheelVelocity.getValue().in(Units.RPM);
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

    var wheelConfig = wheelMotor.getConfigurator();

    wheelConfig.refresh(pidConfig);
    wheelConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = maxAcceleration;

    wheelConfig.apply(pidConfig);
    wheelConfig.apply(mmConfig);
  }

  public void setHoodPos(Rotation2d pos) {
    hoodClosedLoopControl.withPosition(pos.getRotations());
    hoodMotor.setControl(hoodClosedLoopControl);
  }

  public void setRotationPos(Rotation2d pos) {
    rotationClosedLoopControl.withPosition(pos.getRotations());
    rotationMotor.setControl(rotationClosedLoopControl);
  }

  public void configHood(double kP, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    hoodMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    hoodMotor.getConfigurator().apply(slot0Configs);
    hoodMotor.getConfigurator().apply(mmConfigs);
  }

  public void configRotation(double kP, double kD, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    rotationMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;

    rotationMotor.getConfigurator().apply(slot0Configs);
    rotationMotor.getConfigurator().apply(mmConfigs);
  }

  public void setHoodVoltage(double volts) {
    hoodMotor.setControl(hoodOpenLoopControl.withOutput(volts));
  }

  public void setRotationVoltage(double volts) {
    rotationMotor.setControl(rotationOpenLoopControl.withOutput(volts));
  }

  public boolean setHoodNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = hoodMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    hoodMotor.getConfigurator().apply(config);
    return true;
  }

 
    public boolean setRotationNeutralMode(NeutralModeValue value) {
   
    var config = new MotorOutputConfigs();

    var status = rotationMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    rotationMotor.getConfigurator().apply(config);
    return true;
  }
}
