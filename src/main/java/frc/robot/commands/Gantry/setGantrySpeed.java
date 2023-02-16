// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gantry;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class setGantrySpeed extends CommandBase {
  /** Creates a new setLiftSpeed. */
  double goSpeed;
  public setGantrySpeed(double goSpeed ) {
    this.goSpeed = goSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Gantry);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Gantry.setSpeed(goSpeed);



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Gantry.setSpeed(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
