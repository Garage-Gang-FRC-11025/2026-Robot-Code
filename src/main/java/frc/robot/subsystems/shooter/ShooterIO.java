package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class shooterInputs {
    public double wheelsVelocityRPM;
    public double wheelAppliedOutput;
    public double wheelCurrentAmps;
    public double wheelTempCelsius;

    public double hoodAppliedOutput;
    public double hoodCurrentAmps;
    public double hoodTempCelsius;
    public double hoodVelocity;
    public Rotation2d hoodPosition;

    public double rotattionVelocityRPM;
    public double rotattionAppliedOutput;
    public double rotattionCurrentAmps;
    public double rotattionTempCelsius;
    public Rotation2d rotationPosition;
  }

  public default void updateInputs(shooterInputs inputs) {}

  public default void setwheelVoltage(double volts) {}

  public default void sethoodVoltage(double volts) {}

  public default void setrotationVoltage(double volts) {}

  public default void setwheelVel(AngularVelocity vel) {}

  public default void sethoodPos(Rotation2d pos) {}

  public default void setrotationPos(Rotation2d pos) {}

  public default void setrotationVel(AngularVelocity vel) {}

  public default void configwheels(double kV, double kP, double maxAcceleration) {}

  public default void confighood(double kP, double kD, MotionMagicConfigs mmConfigs) {}

  public default void configrotation(double kP, double kD, MotionMagicConfigs mmConfigs) {}

  public default boolean sethoodNeutralMode(NeutralModeValue value) {
    return false;
  }

  public default boolean setrotationNeutralMode(NeutralModeValue value) {
    return false;
  }
}
