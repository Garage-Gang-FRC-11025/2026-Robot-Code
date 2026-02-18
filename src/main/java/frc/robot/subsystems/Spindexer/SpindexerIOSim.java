package frc.robot.subsystems.Spindexer;

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
import frc.robot.Constants.IntakeConstants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.util.TalonFXArmSim;
import frc.robot.util.TalonFXSim;

public class SpindexerIOSim implements SpindexerIO {

  private TalonFXSim rollerMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), SpindexerConstants.ROLLER_GEARING);

  
  private VoltageOut rollerOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage rollerClosedLoopControl = new VelocityVoltage(0);

  public boolean tofActivated = false;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(IntakeInputs inputs) {
    spindexerMotor.update(Constants.kDefaultPeriod);
    inputs.spindexersAppliedOutput = spindexerMotor.getVoltage();
    inputs.spindexersVelocityRPM = spindexerMotor.getVelocity().in(Units.RPM);
  }

  @Override
  public void setSpindexerVoltage(double volts) {
    spindexerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setSpindexerVel(AngularVelocity revPerMin) {
    spindexerMotor.setControl(rollerClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void configSpindexer(double kV, double kP, double maxAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;

    config.Slot0 = slot0Configs;

    spindexerMotor.setConfig(config);
  }


  }

