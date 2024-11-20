// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ScoreCommand extends Command {
  private boolean ended;
  private Elevator elevator;
  private Claw claw;
  private double startTime;
  /** Creates a new ScoreCommand. */
  public ScoreCommand() 
  {
    elevator = Elevator.getInstance();
    claw = Claw.getInstance();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ended = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    elevator.updatePose();
      if(elevator.hasReachedPose(3))
      {
          startTime = Timer.getFPGATimestamp();
          claw.setWheelsOn();
          if(Timer.getFPGATimestamp() - startTime > 1.5)
          {
            ended = true;
          }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevator.setElevatorState(ElevatorState.GROUND);
    claw.setWheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
