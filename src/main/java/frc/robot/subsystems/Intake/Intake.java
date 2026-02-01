package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Intake.IntakeIO.IntakeInputs;
import frc.robot.util.LoggedTunableNumber;


public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeInputs inputs = new IntakeInputs();

   private static final LoggedTunableNumber rKP = new LoggedTunableNumber("Intake/Roller/kP");
      private static final LoggedTunableNumber rKV = new LoggedTunableNumber("Intake/Roller/kV");
      private static final LoggedTunableNumber eKP = new LoggedTunableNumber("Intake/Extender/kP");
      private static final LoggedTunableNumber eKD = new LoggedTunableNumber("Intake/Extender/kD");
      private static final LoggedTunableNumber rollerTargetAccelerationConfig = new LoggedTunableNumber("Intake/Roller/Acceleration");
      private static final LoggedTunableNumber extenderMaxVelocityConfig = new LoggedTunableNumber("Intake/Extender/MaxVel");
      private static final LoggedTunableNumber extenderTargetAccelerationConfig = new LoggedTunableNumber("Intake/Extender/TargetAcceleration");



  static {
    if (Constants.currentMode == Mode.REAL) {
      rKP.initDefault(0.8);
      rKV.initDefault(0.15);
      rollerTargetAccelerationConfig.initDefault(300.0);

      eKP.initDefault(70.0);
      eKD.initDefault(1.6);


      extenderMaxVelocityConfig.initDefault(10);
      extenderTargetAccelerationConfig.initDefault(10);
    } else {
      rKP.initDefault(0.00006);
      rKV.initDefault(0.0002);
      rollerTargetAccelerationConfig.initDefault(0.0);

      eKP.initDefault(120);
      eKD.initDefault(0);

      extenderMaxVelocityConfig.initDefault(40);
      extenderTargetAccelerationConfig.initDefault(80);
    } 
  }

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  public void periodic() {}
}
