// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class setLiftPosition extends CommandBase {
  /** Creates a new setLiftPosition. */

  //temporary variable
  double setPosition;

  boolean finished;

  public setLiftPosition(double setPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    finished = false;

    this.setPosition = setPosition;

    //pushes it into the system
    addRequirements(RobotContainer.s_Lift);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.EndPlaceCommand == true) {
      finished = true;
    } else {
    RobotContainer.s_Lift.setPosition(setPosition);
    finished = false;
    }
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.s_Lift.currentPosition() - setPosition) <= Constants.liftPosTolerance) {
      finished = true;
    }
    
    SmartDashboard.putNumber("debug",Math.abs(RobotContainer.s_Lift.currentPosition() - setPosition));
  }
   
  
  
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
