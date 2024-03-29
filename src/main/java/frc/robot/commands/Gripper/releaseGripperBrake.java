// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gripper;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class releaseGripperBrake extends CommandBase {
  /** Creates a new releaseLiftBreak. */
  public releaseGripperBrake() {
    addRequirements(RobotContainer.s_Gripper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Gripper.setSpeed(0);
    RobotContainer.s_Gripper.setCoastMode();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    RobotContainer.s_Gripper.setBrakeMode();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
