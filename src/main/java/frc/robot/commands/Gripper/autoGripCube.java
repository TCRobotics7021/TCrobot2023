// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class autoGripCube extends CommandBase {
  /** Creates a new setDefaultGripperCommand. */


boolean finished = false;

  public autoGripCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.s_Gripper.pieceSensorBlocked() && RobotContainer.s_Lift.currentPosition()<200) {

     
     

      finished = true; 

    }

    //SmartDashboard.putBoolean("debug2", RobotContainer.s_Gripper.pieceSensorBlocked());


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return finished;
  }
}
