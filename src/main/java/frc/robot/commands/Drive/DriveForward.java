// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveForward extends CommandBase {
 double setSpeed;
 
  /** Creates a new DriveForward. */
  public DriveForward(double setSpeed) {
this.setSpeed = setSpeed;
    addRequirements(RobotContainer.s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.s_Swerve.drive(
          new Translation2d(setSpeed, 0).times(-Constants.Swerve.maxSpeed), 
          0 * Constants.Swerve.maxAngularVelocity, 
          false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
          true
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Swerve.drive(
         new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
        true
      );
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
