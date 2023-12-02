// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoCubeAlign extends CommandBase {
  /** Creates a new NewAutoCubePickup. */
double targetX;
double calcRotation;
boolean finished;

  public AutoCubeAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    RobotContainer.s_Limelight.setPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetX = RobotContainer.s_Limelight.targetX();
    calcRotation = Constants.autoCubePickup_P * targetX;

    if (calcRotation > 0){
      calcRotation = Math.min(Constants.maxAutoRot, calcRotation);
      calcRotation = Math.max(.16,calcRotation);
    }

    if (calcRotation < 0){
      calcRotation = Math.max(-Constants.maxAutoRot, calcRotation);
      calcRotation = Math.min(-.16,calcRotation);
    }
    


      RobotContainer.s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        calcRotation * Constants.Swerve.maxAngularVelocity, 
        false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
        true
    );
    if (targetX < 2 && targetX > -2){
      finished = true;
    }

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
    return finished;
  }
}

