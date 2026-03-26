// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Geometry;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimeShootCommand extends Command {
  /** Creates a new ShooterControl2. */
  private static final LoggedTunableNumber wheelVelocityConfig =
      new LoggedTunableNumber("Shooter/Wheel/Velocity", 3500);

  private static final LoggedTunableNumber elevationAngleConfig =
      new LoggedTunableNumber("Shooter/Elevation/Position", 60);
  private static final LoggedTunableNumber wheelToleranceConfig =
      new LoggedTunableNumber("Shooter/Wheel/Tolerance", 100);
  private static final LoggedTunableNumber elevationToleranceConfig =
      new LoggedTunableNumber("Shooter/Elevation/Tolerance", 5);
  private static final LoggedTunableNumber turretRotationToleranceConfig =
      new LoggedTunableNumber("Shooter/Turret/Tolerance", 10);
  private Shooter shooter;
  private Drive drive;
  private Intake intake;
  private boolean isPrimed = false;

  public PrimeShootCommand(Shooter shooter, Drive drive, Intake intake) {
    this.shooter = shooter;
    this.drive = drive;
    this.intake = intake;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooter.setWheelVel(Units.RPM.of(wheelVelocityConfig.get()));
    shooter.setHoodElevation(Rotation2d.fromDegrees(elevationAngleConfig.get()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d targetTurretRotation = updateTurretRotation();

    Rotation2d targetElevationAngle =
        Rotation2d.fromDegrees(
            Constants.ShooterConstants.HOOD_DISTANCE_ANGLE_TABLE.get(turretHubDistance()));
    double targetFlywheelSpeed =
        Constants.ShooterConstants.FLYWHEEL_DISTANCE_SPEED_TABLE.get(turretHubDistance());

    shooter.setWheelVel(Units.RPM.of(targetFlywheelSpeed));
    shooter.setHoodElevation(targetElevationAngle);

    boolean elevationInPosition =
        withinTolerance(
            targetElevationAngle.getDegrees() - 25,
            shooter.getHoodElevation().getDegrees(),
            elevationToleranceConfig.get());
    boolean turretInPosition =
        withinTolerance(
            targetTurretRotation.getDegrees(),
            shooter.getTurretRotation().getDegrees(),
            turretRotationToleranceConfig.get());
    boolean wheelAtVelocity =
        withinTolerance(
            targetFlywheelSpeed, shooter.getWheelVel().in(Units.RPM), wheelToleranceConfig.get());
    boolean checkExtenderPosition =
        Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees()
            < intake.getExtenderPos().getDegrees();

    final var msg = "target=%s real=%s tolerance=%s okay=%s";
    SmartDashboard.putString(
        "hood elevation",
        msg.formatted(
            targetElevationAngle.getDegrees(),
            shooter.getHoodElevation().getDegrees(),
            elevationToleranceConfig.get(),
            elevationInPosition));
    SmartDashboard.putString(
        "turret rotation",
        msg.formatted(
            targetTurretRotation.getDegrees(),
            shooter.getTurretRotation().getDegrees(),
            turretRotationToleranceConfig.get(),
            turretInPosition));
    SmartDashboard.putString(
        "wheel velocity",
        msg.formatted(
            targetFlywheelSpeed,
            shooter.getWheelVel().in(Units.RPM),
            wheelToleranceConfig.get(),
            wheelAtVelocity));
    SmartDashboard.putString(
        "extender",
        "%s < %s : %s"
            .formatted(
                Constants.IntakeConstants.ExtenderConstants.MIN_REQ_EXTENDER_ANGLE.getDegrees(),
                intake.getExtenderPos().getDegrees(),
                checkExtenderPosition));

    isPrimed = elevationInPosition && turretInPosition && wheelAtVelocity && checkExtenderPosition;

    Logger.recordOutput("ShooterControl2/hoodInPosition", elevationInPosition);
    Logger.recordOutput("ShooterControl2/rotationInPosition", turretInPosition);
    Logger.recordOutput("ShooterControl2/wheelAtPosition", wheelAtVelocity);
    Logger.recordOutput("ShooterControl2/checkExtenderPosition", checkExtenderPosition);
  }

  private Rotation2d updateTurretRotation() {
    final Rotation2d heading =
        Geometry.headingPosition(turretFieldPosition(), FieldConstants.ourHubPosition());
    double targetRotationDegrees = heading.getDegrees() - drive.getRotation().getDegrees();
    Logger.recordOutput(
        "RotationTargetPositionBefore", Rotation2d.fromDegrees(targetRotationDegrees));

    System.out.println("targetRotationDeg = " + targetRotationDegrees);

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
    shooter.setHoodElevation(new Rotation2d(0));
    isPrimed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isPrimed() {
    return isPrimed;
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
