package frc.robot.subsystems.Spindexer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private final SpindexerIO spintakerIO;
  private final SpindexerautoLogged inputs = new SpindexerInputsAutoLogged();

  private static final LoggedTunableNumber rKP = new LoggedTunableNumber("Intake/Roller/kP");
  private static final LoggedTunableNumber rKV = new LoggedTunableNumber("Intake/Roller/kV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig =
      new LoggedTunableNumber("Intake/Roller/Acceleration");
  private static final LoggedTunableNumber extenderMaxVelocityConfig =
      new LoggedTunableNumber("Intake/Extender/MaxVel");
  private static final LoggedTunableNumber extenderTargetAccelerationConfig =
      new LoggedTunableNumber("Intake/Extender/TargetAcceleration");

  static {
    if (Constants.currentMode == Mode.REAL) {
      rKP.initDefault(0.8);
      rKV.initDefault(0.15);
      TargetAccelerationConfig.initDefault(300.0);

      eKP.initDefault(70.0);
      eKD.initDefault(1.6);
  }


  public void setRollerVoltage(double volts) {
    spindexerIO.setRollerVoltage(volts);
  }


  public void setRollerVel(AngularVelocity vel) {
    spindexerIO.setRollerVel(vel);
  }



