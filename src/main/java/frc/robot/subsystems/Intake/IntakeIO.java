package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public interface IntakeIO {

    class IntakeInputs  {
        public double rollersVelocityRPM;
    public double rollersAppliedOutput;
    public double rollersCurrentAmps;
    public double rollersTempCelsius;
    public double extenderPositionInches;
    public double extenderAppliedOutput;
    public double extenderCurrentAmps;
    public double extenderTempCelsius;
    }

public default void updateInputs(IntakeInputs inputs) {}

public default void setRollerVoltage(double volts) {}

public default void setExtenderVoltage(double volts) {}

public default void setRollerVel(AngularVelocity vel) {}

public default void setExtenderPos(Distance length) {}

public default void configRollers(double kV, double kP, double maxAcceleration) {}

public default void configExtender(double kP, double kD, MotionMagicConfigs mmConfigs) {}

public default boolean setExtenderNeutralMode(NeutralModeValue value) {
    return false;
  }

}
