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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterControl extends Command {
  /** Creates a new ShooterControl2. */
  private static final LoggedTunableNumber wheelVelocityConfig =
      new LoggedTunableNumber("Shooter/Wheel/Velocity", 3500);

  private static final LoggedTunableNumber elevatorVelocityConfig =
      new LoggedTunableNumber("Elevator/Elevator/Velocity", 400);
  private static final LoggedTunableNumber hoodPositionConfig =
      new LoggedTunableNumber("Shooter/Hood/Position", 60);
  private static final LoggedTunableNumber wheelToleranceConfig =
      new LoggedTunableNumber("Shooter/Wheel/Tolerance", 30);
  private static final LoggedTunableNumber hoodToleranceConfig =
      new LoggedTunableNumber("Shooter/hood/Tolerance", 5);
  private static final LoggedTunableNumber rotationToleranceConfig =
      new LoggedTunableNumber("Shooter/Rotation/Tolerance", 5);
  private Shooter shooter;
  private Elevator elevator;
  private Drive drive;
  private Intake intake;

  public ShooterControl(Shooter shooter, Elevator elevator, Drive drive, Intake intake) {
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
    Rotation2d targetRotationPos = updateRotation();

    Rotation2d targetHoodAngle =
        Rotation2d.fromDegrees(
            Constants.ShooterConstants.HOOD_DISTANCE_ANGLE_TABLE.get(turretHubDistance()));
    double targetFlywheelSpeed =
        Constants.ShooterConstants.FLYWHEEL_DISTANCE_SPEED_TABLE.get(turretHubDistance());

    shooter.setWheelVel(Units.RPM.of(wheelVelocityConfig.get()));
    shooter.setHoodPos(Rotation2d.fromDegrees(hoodPositionConfig.get()));
    boolean hoodInPosition =
        withinTolerance(
            hoodPositionConfig.get(), shooter.getHoodPos().getDegrees(), hoodToleranceConfig.get());
    boolean rotationInPosition =
        withinTolerance(
            targetRotationPos.getDegrees(),
            shooter.getRotationPos().getDegrees(),
            rotationToleranceConfig.get());
    boolean wheelAtVelocity =
        withinTolerance(
            wheelVelocityConfig.get(),
            shooter.getWheelVel().in(Units.RPM),
            wheelToleranceConfig.get());
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

  private Rotation2d updateRotation() {
    final Rotation2d heading =
        Geometry.headingPosition(turretFieldPosition(), FieldConstants.ourHubPosition());
    double targetRotationDegrees = heading.getDegrees() - drive.getRotation().getDegrees();

    Rotation2d targetRotationPos = Rotation2d.fromDegrees(targetRotationDegrees);
    shooter.setRotationPos(targetRotationPos);
    Logger.recordOutput("RotationTargetPosition", targetRotationPos);
    return targetRotationPos;
  }

  private boolean withinTolerance(double targetState, double currentState, double tolerance) {
    return Math.abs(currentState - targetState) < tolerance;
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
    Translation2d turretFieldPosition = turretTranslation2d.plus(drive.getPose().getTranslation());
    return turretFieldPosition;
  }

  private double turretHubDistance() {
    double turretHubDistance =
        Constants.FieldConstants.ourHubPosition().getDistance(turretFieldPosition());
    Logger.recordOutput("ShooterControl2/HubDistance", turretHubDistance);
    return turretHubDistance;
  }
}
