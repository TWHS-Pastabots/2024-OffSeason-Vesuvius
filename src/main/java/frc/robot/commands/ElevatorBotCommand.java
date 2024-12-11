// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorBotCommand extends Command {
  private Elevator elevator;
  private boolean ended;
  /** Creates a new ElevatorGoBot. */
  public ElevatorBotCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = Elevator.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ended = false;
    elevator.setElevatorState(ElevatorState.BOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.updatePose();
    if(elevator.getElevatorState() == ElevatorState.BOT && elevator.hasReachedPose(3))
    ended = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
