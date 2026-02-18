package frc.robot.subsystems.Spindexer;

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
import frc.robot.Constants.ExtenderConstants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;

public class SpindexerIOReal implements SpindexerIO {

  private final TalonFX rollerMotor = new TalonFX(Constants.CanIDs.INTAKE_ROLLER_CAN_ID);

  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerDeviceTemp;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Temperature> extenderDeviceTemp;
  private final StatusSignal<Voltage> extenderAppliedVoltage;
  private final StatusSignal<Angle> extenderAngle;
  private final StatusSignal<Current> extenderCurrent;

  private final VoltageOut rollerOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut extenderOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage rollerClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage extenderClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public SpindexerIOReal() {
    // Motor config
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimitConfig = new CurrentLimitsConfigs();

    rollerCurrentLimitConfig.SupplyCurrentLimit = SpindexerConstants.SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    rollerCurrentLimitConfig.StatorCurrentLimit = SpindexerConstants.STATOR_CURRENT_LIMIT;
    rollerCurrentLimitConfig.StatorCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimitConfig;

    rollerConfig.Feedback.SensorToMechanismRatio = SpindexerConstants.ROLLER_GEARING;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerConfig.Voltage.SupplyVoltageTimeConstant = SpindexerConstants.SUPPLY_VOLTAGE_TIME;

    rollerMotor.getConfigurator().apply(rollerConfig);

    // Status signals

    rollerCurrent = rollerMotor.getStatorCurrent();
    rollerDeviceTemp = rollerMotor.getDeviceTemp();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerVelocity = rollerMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SpindexerInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerAppliedVoltage,
        rollerCurrent,
        rollerDeviceTemp,
        rollerVelocity,
        extenderAngle,
        extenderCurrent,
        extenderAppliedVoltage,
        extenderDeviceTemp);

    inputs.rollersCurrentAmps = rollerCurrent.getValue().in(Units.Amps);
    inputs.rollersTempCelsius = rollerDeviceTemp.getValue().in(Units.Celsius);
    inputs.rollersAppliedOutput = rollerAppliedVoltage.getValue().in(Units.Volts);
    inputs.rollersVelocityRPM = rollerVelocity.getValue().in(Units.RPM);
  }

  @Override
  public void setSpindexerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setSpindexerVel(AngularVelocity vel) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(vel.in(Units.RotationsPerSecond)));
  }

  @Override
  public void configSpindexer(double kV, double kP, double maxAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var rollerConfig = rollerMotor.getConfigurator();

    rollerConfig.refresh(pidConfig);
    rollerConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = maxAcceleration;

    rollerConfig.apply(pidConfig);
    rollerConfig.apply(mmConfig);
  }

}
