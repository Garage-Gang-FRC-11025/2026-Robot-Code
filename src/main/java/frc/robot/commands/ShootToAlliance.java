// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Geometry;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootToAlliance extends Command {
  /** Creates a new ShootToalliance. */
  private static final LoggedTunableNumber wheelVelocityConfig =
      new LoggedTunableNumber("Shooter/Wheel/Velocity", 350);

  private static final LoggedTunableNumber elevatorVelocityConfig =
      new LoggedTunableNumber("Elevator/Elevator/Velocity", 350);
  private static final LoggedTunableNumber hoodPositionConfig =
      new LoggedTunableNumber("Shooter/Hood/Position", 60);

  private Shooter shooter;
  private Elevator elevator;
  private Drive drive;
  private Intake intake;

  public ShootToAlliance(Shooter shooter, Elevator elevator, Drive drive, Intake intake) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.drive = drive;
    this.intake = intake;
    addRequirements(shooter, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setWheelVel(Units.RPM.of(wheelVelocityConfig.get()));
    shooter.setHoodPos(Rotation2d.fromDegrees(hoodPositionConfig.get()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d targetAllianceHoodAngle =
        Rotation2d.fromDegrees(
            Constants.ShooterConstants.HOOD_ALLIANCE_DISTANCE_ANGLE_TABLE.get(
                turretAllianceDistance()));
    double targetAllianceFlywheelSpeed =
        Constants.ShooterConstants.FLYWHEEL_ALLIANCE_DISTANCE_SPEED_TABLE.get(
            turretAllianceDistance());
    Rotation2d targetAllianceRotationPos =
        Geometry.headingPosition(turretFieldPosition(), FieldConstants.ourAlliancePosition())
            .minus(drive.getRotation());
    shooter.setRotationPos(targetAllianceRotationPos);
    boolean hoodInPosition =
        withinTolerance(targetAllianceHoodAngle.getDegrees(), shooter.getHoodPos().getDegrees(),
5);
    boolean rotationInPosition =
        withinTolerance(
            targetAllianceRotationPos.getDegrees(), shooter.getRotationPos().getDegrees(), 5);
    boolean wheelAtVelocity =
        withinTolerance(targetAllianceFlywheelSpeed, shooter.getWheelVel().in(Units.RPM), 10);

    boolean checkExtenderPosition =
        Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees()
            < intake.getExtenderPos().getDegrees();

    if (hoodInPosition && rotationInPosition && wheelAtVelocity && checkExtenderPosition)
      elevator.setElevatorVel(Units.RPM.of(elevatorVelocityConfig.get()));

    Logger.recordOutput("ShooterControl2/hoodInPosition", hoodInPosition);
    Logger.recordOutput("ShooterControl2/rotationInPosition", rotationInPosition);
    Logger.recordOutput("ShooterControl2/wheelAtPosition", wheelAtVelocity);
    Logger.recordOutput("ShooterControl2/checkExtenderPosition", checkExtenderPosition);
  }

  private boolean withinTolerance(double targetState, double currentState, double tolerance) {
    return Math.abs(currentState - targetState) < tolerance;
  }

  public Rotation2d updateAllianceRotation() {
    final Rotation2d Allianceheading =
        Geometry.headingPosition(turretFieldPosition(), FieldConstants.ourAlliancePosition());
    double targetAllianceRotationDegrees = Allianceheading.getDegrees() -
drive.getRotation().getDegrees();
    Logger.recordOutput(
        "RotationTargetPositionBefore", Rotation2d.fromDegrees(targetAllianceRotationDegrees));

    System.out.println("targetRotationDeg = " + targetAllianceRotationDegrees);
     if (targetAllianceRotationDegrees < -170) {
       targetAllianceRotationDegrees = targetAllianceRotationDegrees + 360;
     } else if (targetAllianceRotationDegrees > 220) {
       targetAllianceRotationDegrees = targetAllianceRotationDegrees - 360;
    }
    return Allianceheading;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setWheelVel((Units.RPM.of(0)));
    shooter.setHoodPos(new Rotation2d(0));
    elevator.setElevatorVel(Units.RPM.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Translation2d turretFieldPosition() {
    Translation2d turretTranslation2d =
        Constants.ShooterConstants.TURRET_TRANSLATION.rotateBy(drive.getRotation());
    Translation2d turretFieldPosition =
turretTranslation2d.plus(drive.getPose().getTranslation());
    return turretFieldPosition;
  }

  private double turretAllianceDistance() {
    double turretAllianceDistance =
        Constants.FieldConstants.ourAlliancePosition().getDistance(turretFieldPosition());
    Logger.recordOutput("ShooterControl2/AllianceDistance", turretAllianceDistance);
    return turretAllianceDistance;
  }
}
