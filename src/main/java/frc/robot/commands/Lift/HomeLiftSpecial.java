// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HomeLiftSpecial extends CommandBase {
  /** Creates a new HomeLiftSpecial. */
  boolean finished; 
  boolean Trigger1;
  public HomeLiftSpecial() {
    finished = false;
    Trigger1 = false;
    addRequirements(RobotContainer.s_Lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Lift.setSpeed(Constants.setSpeedForLiftHome);
    RobotContainer.s_Lift.calibrateEncoder(400);  
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Lift.atTopLimit()) {
      RobotContainer.s_Lift.calibrateEncoder(Constants.liftUpperLimitSwitchPos);
      finished = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Lift.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
