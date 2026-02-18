package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.RollerConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX elevatorMotor = new TalonFX(Constants.CanIDs.ELEVATOR_ROLLER_CAN_ID);

  private final StatusSignal<Current> elevatorCurrent;
  private final StatusSignal<Temperature> elevatorDeviceTemp;
  private final StatusSignal<Voltage> elevatorAppliedVoltage;
  private final StatusSignal<AngularVelocity> elevatorVelocity;

  private final VoltageOut elevatorOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final VoltageOut extenderOpenLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final MotionMagicVelocityVoltage elevatorClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private final MotionMagicVoltage extenderClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(false);

  public ElevatorIOReal() {
    // Motor config
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs elevatorCurrentLimitConfig = new CurrentLimitsConfigs();

    elevatorCurrentLimitConfig.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT;
    elevatorCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimitConfig.StatorCurrentLimit = RollerConstants.STATOR_CURRENT_LIMIT;
    elevatorCurrentLimitConfig.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimitConfig;

    elevatorConfig.Feedback.SensorToMechanismRatio = RollerConstants.ROLLER_GEARING;
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    elevatorConfig.Voltage.SupplyVoltageTimeConstant = RollerConstants.SUPPLY_VOLTAGE_TIME;

    elevatorMotor.getConfigurator().apply(elevatorConfig);

    // Status signals

    elevatorCurrent = elevatorMotor.getStatorCurrent();
    elevatorDeviceTemp = elevatorMotor.getDeviceTemp();
    elevatorAppliedVoltage = elevatorMotor.getMotorVoltage();
    elevatorVelocity = elevatorMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, elevatorAppliedVoltage, elevatorCurrent, elevatorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, elevatorDeviceTemp);

    elevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorAppliedVoltage, elevatorCurrent, elevatorDeviceTemp, elevatorVelocity);

    inputs.elevatorsCurrentAmps = elevatorCurrent.getValue().in(Units.Amps);
    inputs.elevatorsTempCelsius = elevatorDeviceTemp.getValue().in(Units.Celsius);
    inputs.elevatorsAppliedOutput = elevatorAppliedVoltage.getValue().in(Units.Volts);
    inputs.elevatorsVelocityRPM = elevatorVelocity.getValue().in(Units.RPM);
  }

  @Override
  public void setRollerVoltage(double volts) {
    elevatorMotor.setControl(elevatorOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVel(AngularVelocity vel) {
    elevatorMotor.setControl(
        elevatorClosedLoopControl.withVelocity(vel.in(Units.RotationsPerSecond)));
  }

  @Override
  public void configRollers(double kV, double kP, double maxAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var elevatorConfig = elevatorMotor.getConfigurator();

    elevatorConfig.refresh(pidConfig);
    elevatorConfig.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = maxAcceleration;

    elevatorConfig.apply(pidConfig);
    elevatorConfig.apply(mmConfig);
  }
}
