// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class releaseAllBreaks extends CommandBase {
  /** Creates a new releaseAllBreaks. */
  public releaseAllBreaks() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.s_Gantry);
    addRequirements(RobotContainer.s_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    RobotContainer.s_Gantry.setSpeed(0);
    RobotContainer.s_Gantry.setCoastMode();
    RobotContainer.s_Lift.setSpeed(0);
    RobotContainer.s_Lift.setCoastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    RobotContainer.s_Gantry.setBrakeMode();
    RobotContainer.s_Lift.setBrakeMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
