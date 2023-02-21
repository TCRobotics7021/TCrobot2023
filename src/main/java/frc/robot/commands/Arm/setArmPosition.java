// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class setArmPosition extends CommandBase {
  /** Creates a new setLiftPosition. */

  //temporary variable
  double setPosition;

  boolean finished;

  public setArmPosition(double setPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    finished = false;

    this.setPosition = setPosition;

    //pushes it into the system
    addRequirements(RobotContainer.s_Arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Arm.setPosition(setPosition);
    finished = false;
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.s_Arm.currentPosition() - setPosition) <= Constants.ArmPosTolerance) {
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
