package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.LoggedTunableNumber;

public class ShootCommand extends Command {
  private Elevator elevator;
  private PrimeShootCommand primeShootCommand;

  private static final LoggedTunableNumber elevatorVelocityConfig =
      new LoggedTunableNumber("Elevator/Elevator/Velocity", 400);

  public ShootCommand(Elevator elevator, PrimeShootCommand primeShootCommand) {
    this.elevator = elevator;
    this.primeShootCommand = primeShootCommand;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    if (primeShootCommand.isPrimed())
      elevator.setElevatorVel(Units.RPM.of(elevatorVelocityConfig.get()));
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorVel(Units.RPM.of(0));
  }
}
