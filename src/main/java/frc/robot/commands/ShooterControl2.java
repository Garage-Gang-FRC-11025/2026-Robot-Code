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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Geometry;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterControl2 extends Command {
  /** Creates a new ShooterControl2. */
  private static final LoggedTunableNumber wheelVelocityConfig =
      new LoggedTunableNumber("Shooter/Wheel/Velocity", 3500);

  private static final LoggedTunableNumber elevatorVelocityConfig =
      new LoggedTunableNumber("Elevator/Elevator/Velocity", 400);
  private static final LoggedTunableNumber hoodPositionConfig =
      new LoggedTunableNumber("Shooter/Hood/Position", 60);
  private static final LoggedTunableNumber rotationPositionConfig =
      new LoggedTunableNumber("Shooter/Rotation/Position");

  private Shooter shooter;
  private Elevator elevator;
  private Drive drive;

  public ShooterControl2(Shooter shooter, Elevator elevator, Drive drive) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.drive = drive;
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
    Rotation2d targetHubHoodAngle =
        Rotation2d.fromDegrees(
            Constants.ShooterConstants.HOOD_HUB_DISTANCE_ANGLE_TABLE.get(turretHubDistance()));
    double targetHubFlywheelSpeed =
        Constants.ShooterConstants.FLYWHEEL_HUB_DISTANCE_SPEED_TABLE.get(turretHubDistance());
    Rotation2d targetRotationPos =
        Geometry.headingPosition(turretFieldPosition(), FieldConstants.ourHubPosition())
            .minus(drive.getRotation());
    shooter.setRotationPos(targetRotationPos);
    boolean hoodInPosition =
        withinTolerance(hoodPositionConfig.get(), shooter.getHoodPos().getDegrees(), 5);
    boolean rotationInPosition =
        withinTolerance(targetRotationPos.getDegrees(), shooter.getRotationPos().getDegrees(), 5);
    boolean wheelAtVelocity =
        withinTolerance(wheelVelocityConfig.get(), shooter.getWheelVel().in(Units.RPM), 10);

    if (hoodInPosition && rotationInPosition && wheelAtVelocity)
      elevator.setElevatorVel(Units.RPM.of(elevatorVelocityConfig.get()));

    Logger.recordOutput("ShooterControl2/hoodInPosition", hoodInPosition);
    Logger.recordOutput("ShooterControl2/rotationInPosition", rotationInPosition);
    Logger.recordOutput("ShooterControl2/wheelAtPosition", wheelAtVelocity);
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
