// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class MoveToPosReletiveToTarget extends CommandBase {

  /** Creates a new MoveToPosReletiveToTarget. */
  double relativeX;
  double relativeY; 
  double targetR;

  double calcStrafe;
  double calcTranslation;
  double calcRotation;

  double currentX;
  double currentY;
  double currentR;

  double errorX;
  double errorY;
  double errorR;

  double calcMagnitude; 

  boolean finished;

  double degrees;

  double ratio;





  public MoveToPosReletiveToTarget(double relativeX, double relativeY, double targetR) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.relativeX = relativeX;
    this.relativeY = relativeY;
    this.targetR = targetR;
    addRequirements(RobotContainer.s_Swerve);
    addRequirements(RobotContainer.s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //currentR = -RobotContainer.s_Limelight.tagRelativeRPos();
    currentR = RobotContainer.s_Swerve.getYaw().getDegrees();
    currentX = RobotContainer.s_Limelight.tagRelativeXPos();
    currentY = RobotContainer.s_Limelight.tagRelativeYPos();
    RobotContainer.s_Swerve.resetOdometry(new Pose2d(currentX, currentY, new Rotation2d(Math.toRadians(currentR))));
    finished = false;
    
    SmartDashboard.putNumber("Starting X", currentX); 
    SmartDashboard.putNumber("Starting Y", currentY); 
    SmartDashboard.putNumber("Starting R", currentR); 
    
    if (currentR > 30 || currentR <-30) {
      RobotContainer.EndPlaceCommand = true;
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = RobotContainer.s_Swerve.getPose().getX();
    currentY = RobotContainer.s_Swerve.getPose().getY();
    // currentR = RobotContainer.s_Swerve.getPose().getRotation().getDegrees();
    currentR = RobotContainer.s_Swerve.getPose().getRotation().getDegrees();


    SmartDashboard.putNumber("Current X", currentX); 
    SmartDashboard.putNumber("Current Y", currentY); 
    SmartDashboard.putNumber("Current R", currentR); 

    errorX = Math.abs(relativeX - currentX);
    errorY = Math.abs(relativeY - currentY);
    errorR = -(targetR - currentR);

    //Finds shortest path for rotation
    if (errorR > 180){
      errorR = errorR -360;
    } 
    if (errorR < -180){
      errorR = errorR + 360;
    }

    
    calcTranslation = Constants.autonomousMove_P * (relativeX - currentX);
    calcStrafe = Constants.autonomousMove_P * (relativeY - currentY);
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));
    calcRotation = Constants.autoRotate_P * errorR;

  

    if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
      calcTranslation = Math.min(Constants.maxSpeedPos,calcTranslation);
      //calcTranslation = Math.min(.5, calcTranslation);
    }
  
    if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
      //calcTranslation = Math.min(-.2,calcTranslation);
      calcTranslation = Math.max(-Constants.maxSpeedPos, calcTranslation);
    }

    if (errorX < Constants.errorTolerance){
      calcTranslation = 0;
    }



    if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
      calcStrafe = Math.min(Constants.maxSpeedPos,calcStrafe);
      //calcStrafe = Math.min(.5, calcStrafe);
    }

    if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
      //calcStrafe = Math.min(-.2,calcStrafe);
      calcStrafe = Math.max(-Constants.maxSpeedPos, calcStrafe);
    }
  
    if (errorY < Constants.errorTolerance){
      calcStrafe = 0;
    }

    if (calcRotation > 0){
      calcRotation = Math.min(Constants.maxAutoRot, calcRotation);
    }
    
    if (calcRotation < 0){
      calcRotation = Math.max(-Constants.maxAutoRot, calcRotation);
    }


    if (Math.abs(errorR) > Constants.autoRotateTolerance){
      if (calcRotation > 0){
        calcRotation = Math.max(Constants.minAutoRot, calcRotation);
      }
      
      if (calcRotation < 0){
        calcRotation = Math.min(-Constants.minAutoRot, calcRotation);
      }
    }


    if (calcMagnitude <= Constants.minSpeedPos && calcStrafe != 0) {
      ratio = calcTranslation / calcStrafe;
      calcStrafe = Math.sqrt(Math.pow(Constants.minSpeedPos, 2) / (Math.pow(ratio, 2) + 1)) * Math.signum(calcStrafe);
      calcTranslation = Math.abs(calcStrafe * ratio) * Math.signum(calcTranslation);
  
    }
    if(calcStrafe == 0 && calcTranslation != 0){
      calcTranslation = Math.max(Constants.minSpeedPos,Math.abs(calcTranslation)) * Math.signum(calcTranslation);
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
      calcRotation * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );

  //&& Math.abs(errorR) <= Constants.autoRotateTolerance
   if (errorX <= Constants.errorTolerance && errorY <= Constants.errorTolerance && Math.abs(errorR) <= Constants.autoRotateTolerance){
      finished = true;
    }
    SmartDashboard.putNumber("Error R", errorR);
    SmartDashboard.putNumber("Error X", errorX);
    SmartDashboard.putNumber("Error Y", errorX);
    SmartDashboard.putNumber("Calc Translation",calcTranslation);
    SmartDashboard.putNumber("Calc Strafe",calcStrafe);
    SmartDashboard.putNumber("Calc Rotate", calcRotation);
    
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
