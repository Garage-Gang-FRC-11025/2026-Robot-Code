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
import frc.robot.Constants.ShooterConstants.WheelConstants;
import frc.robot.util.TalonFXArmSim;
import frc.robot.util.TalonFXSim;

public class ShooterIOSim implements ShooterIO {

  private TalonFXSim rollerMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), RollerConstants.ROLLER_GEARING, RollerConstants.ROLLER_MOI);

  private TalonFXArmSim HoodSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              frc.robot.Constants.IntakeConstants.HoodConstants.GEAR_RATIO,
              HoodConstants.HOOD_MOI,
              HoodConstants.HOOD_LENGTH.in(Units.Meters),
              HoodConstants.MIN_HOOD_ANGLE.getRadians(),
              HoodConstants.MAX_HOOD_ANGLE.getRadians(),
              false,
              0));

  private VoltageOut rollerOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage rollerClosedLoopControl = new VelocityVoltage(0);

  private VoltageOut HoodOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage HoodClosedLoopControl = new MotionMagicVoltage(0);

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeInputs inputs) {
    rollerMotor.update(Constants.kDefaultPeriod);
    inputs.rollersAppliedOutput = rollerMotor.getVoltage();
    inputs.rollersVelocityRPM = rollerMotor.getVelocity().in(Units.RPM);

    HoodSim.update(Constants.kDefaultPeriod);

    inputs.HoodPosition = new Rotation2d(HoodSim.getPosition());
    inputs.HoodAppliedOutput = HoodSim.getVoltage().in(Units.Volts);
    inputs.HoodVelocity = HoodSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVel(AngularVelocity revPerMin) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void configRollers(double kV, double kP, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;

    rollerMotor.setConfig(config);
  }

  @Override
  public void setHoodVoltage(double volts) {
    HoodSim.setControl(HoodOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setHoodPos(Rotation2d angle) {
    HoodSim.setControl(HoodClosedLoopControl.withPosition(angle.getRotations()));
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
}
