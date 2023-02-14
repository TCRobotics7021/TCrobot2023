// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HomeArm extends CommandBase {
  /** Creates a new HomeArm. */
  boolean finished; 
  boolean Trigger1;
  public HomeArm() {
    finished = false;
    Trigger1 = false;
    addRequirements(RobotContainer.s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Arm.setSpeed(Constants.setSpeedForArmHome);
    RobotContainer.s_Arm.calibrateEncoder(Constants.ArmLowerLimit);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.s_Arm.atTopLimit()) {
      Trigger1 = true;
      RobotContainer.s_Arm.setSpeed(.05);
     }
     if (Trigger1 == true && !RobotContainer.s_Arm.atTopLimit()) {
      finished = true; 
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     RobotContainer.s_Arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
