
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gantry;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class setGantryPosition extends CommandBase {
  /** Creates a new setLiftPosition. */

  //temporary variable
  double setPosition;

  boolean finished = false;

  public setGantryPosition(double setPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.setPosition = setPosition;

    //pushes it into the system
    addRequirements(RobotContainer.s_Gantry);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.EndPlaceCommand == true) {
      finished = true;
    } else {
    RobotContainer.s_Gantry.setPosition(setPosition);
    finished = false;
    }
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.s_Gantry.currentPosition() - setPosition) <= Constants.GantryPosTolerance) {
      finished = true;
    }
   
    
 
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