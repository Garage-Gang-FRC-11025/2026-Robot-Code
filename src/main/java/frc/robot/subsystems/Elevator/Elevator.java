package frc.robot.subsystems.Elevator;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO ElevatorIO;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private static final LoggedTunableNumber eleKP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber eleKV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber elevatorTargetAccelerationConfig =
      new LoggedTunableNumber("Elevator/Acceleration");

  static {
    if (Constants.currentMode == Mode.REAL) {
      eleKP.initDefault(0.8);
      eleKV.initDefault(0.15);
      elevatorTargetAccelerationConfig.initDefault(300.0);
    }
  }

  public Elevator(ElevatorIO ElevatorIO) {
    this.ElevatorIO = ElevatorIO;
  }

  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    int hc = hashCode();
    ElevatorIO.updateInputs(inputs);
  }

  public void setElevatorVoltage(double volts) {
    ElevatorIO.setElevatorVoltage(volts);
  }

  public void setElevatorVel(AngularVelocity vel) {
    ElevatorIO.setElevatorVel(vel);
  }
}
