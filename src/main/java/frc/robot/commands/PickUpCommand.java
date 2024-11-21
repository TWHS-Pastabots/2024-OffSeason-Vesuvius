// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class PickUpCommand extends Command {
  private boolean ended;
  private Claw claw;
  private double startTime;
  /** Creates a new PickUpCommand. */
  public PickUpCommand() {
    claw = Claw.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ended = false;
    startTime = Timer.getFPGATimestamp();
    claw.setWheelsReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Timer.getFPGATimestamp() - startTime > 4)
    {
      ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setWheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
