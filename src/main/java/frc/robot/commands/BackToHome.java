// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BackToHome extends CommandBase {
  Boolean finished;

  double currentX;
  double currentY;

  double calcTranslation;
  double calcStrafe;

  double odometryX;
  double odometryY;

  double calcMagnitude;

  double errorX;
  double errorY;

  public BackToHome() {
    addRequirements(RobotContainer.s_Swerve);
    finished = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = RobotContainer.s_Swerve.getPose().getX();
    currentY = RobotContainer.s_Swerve.getPose().getY();

    calcTranslation = Constants.autonomousMove_P * Math.abs((0 - currentX));
    calcStrafe = Constants.autonomousMove_P * Math.abs((0 - currentY));
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));


    errorX = Math.abs(0 - currentX);
    errorY = Math.abs(0 - currentY);

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );
  
  if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
    calcTranslation = Math.min(Constants.maxSpeedPos,calcTranslation);
  }

  if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
    calcTranslation = Math.max(-Constants.maxSpeedPos, calcTranslation);
  }
  
  if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
    calcStrafe = Math.min(Constants.maxSpeedPos,calcStrafe);
  }

  if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
    calcStrafe = Math.max(-Constants.maxSpeedPos, calcStrafe);
  }

    if (errorX <= .05 && errorY <= .05) {
      finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotContainer.s_Swerve.resettempOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    return false;
  }
}
