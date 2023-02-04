// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class MoveToPosReletiveToTarget extends CommandBase {

  /** Creates a new MoveToPosReletiveToTarget. */
  double relativeX;
  double relativeY; 

  double calcStrafe;
  double calcTranslation;
  double calcRotation;

  double currentX;
  double currentY;

  double errorX;
  double errorY;

  double calcMagnitude; 

  boolean finished;

  double degrees;





  public MoveToPosReletiveToTarget(double relativeX, double relativeY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.relativeX = relativeX;
    this.relativeY = relativeY;
    finished = false;
    addRequirements(RobotContainer.s_Swerve);
    addRequirements(RobotContainer.s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    degrees = RobotContainer.s_Limelight.tagRelativeRPos();
    currentX = RobotContainer.s_Limelight.tagRelativeXPos();
    currentY = RobotContainer.s_Limelight.tagRelativeYPos();
    RobotContainer.s_Swerve.resettempOdometry(new Pose2d(currentX, currentY, new Rotation2d(degrees)));
    RobotContainer.s_Swerve.setGyro(degrees);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = RobotContainer.s_Swerve.getPose().getX();
    currentY = RobotContainer.s_Swerve.getPose().getY();

    errorX = Math.abs(relativeX - currentX);
    errorY = Math.abs(relativeY - currentY);

    calcTranslation = Constants.Swerve.autonomousMove_P * (relativeX - currentX);
    calcStrafe = Constants.Swerve.autonomousMove_P * (relativeY - currentY);
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));


  

    if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
      calcTranslation = Math.min(Constants.maxSpeedPos,calcTranslation);
      //calcTranslation = Math.min(.5, calcTranslation);
    }
  
    if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
      //calcTranslation = Math.min(-.2,calcTranslation);
      calcTranslation = Math.max(-Constants.maxSpeedPos, calcTranslation);
    }
    

    if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
      calcStrafe = Math.max(Constants.maxSpeedPos,calcStrafe);
      //calcStrafe = Math.min(.5, calcStrafe);
    }

    if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
      //calcStrafe = Math.min(-.2,calcStrafe);
      calcStrafe = Math.max(-Constants.maxSpeedPos, calcStrafe);
    }
  

    if (calcMagnitude <= Constants.minSpeedPos) {
      if (Math.abs(calcTranslation) < Math.abs(calcStrafe)) {
        calcStrafe = Constants.minSpeedPos * Math.signum(calcStrafe);
      }
      if (Math.abs(calcTranslation) >= Math.abs(calcStrafe)) {
        calcTranslation = Constants.minSpeedPos;
      }
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );
  if (errorX <= .05 && errorY <= .05){
    finished = true;
  }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
    );


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    return finished;
  }
}
