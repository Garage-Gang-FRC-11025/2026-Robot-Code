package frc.robot.subsystems.Spindexer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  class IntakeInputs {
    public double rollersVelocityRPM;
    public double rollersAppliedOutput;
    public double rollersCurrentAmps;
    public double rollersTempCelsius;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setSpindexerVoltage(double volts) {}

  public default void setSpindexerVel(AngularVelocity vel) {}

  public default void configSpindexer(double kV, double kP, double maxAcceleration) {}

}
