// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gantry;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HomeGantry extends CommandBase {
  /** Creates a new HomeGantry. */
  boolean finished; 
  boolean Trigger1;
  public HomeGantry() {
    finished = false;
    Trigger1 = false;
    addRequirements(RobotContainer.s_Gantry);
    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Gantry.setSpeed(Constants.setSpeedforGantryHome);
    RobotContainer.s_Gantry.calibrateEncoder(Constants.GantryUpperLimit);  
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Gantry.atBottomLimit()) {
      finished = true; 
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     RobotContainer.s_Gantry.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
