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

  private static final LoggedTunableNumber eKP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber eKV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber elevatorTargetAccelerationConfig =
      new LoggedTunableNumber("Elevator/Acceleration");

  static {
    if (Constants.currentMode == Mode.REAL) {
      eKP.initDefault(0.8);
      eKV.initDefault(0.15);
      elevatorTargetAccelerationConfig.initDefault(300.0);

      elevatorTargetAccelerationConfig.initDefault(0.0);
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

  public void setRollerVoltage(double volts) {
    ElevatorIO.setRollerVoltage(volts);
  }

  public void setRollerVel(AngularVelocity vel) {
    ElevatorIO.setRollerVel(vel);
  }
}
