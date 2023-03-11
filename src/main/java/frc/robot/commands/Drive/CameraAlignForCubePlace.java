// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CameraAlignForCubePlace extends CommandBase {

  /** Creates a new MoveToPosReletiveToTarget. */
  double targetX;
  double targetY; 
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





  public CameraAlignForCubePlace(){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
    addRequirements(RobotContainer.s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //currentR = -RobotContainer.s_Limelight.tagRelativeRPos();
    currentR = RobotContainer.s_Swerve.getYaw().getDegrees();
    currentX = RobotContainer.s_Limelight.targetX();
    currentY = RobotContainer.s_Limelight.targetY();
    targetR=0;
    targetX= Constants.limeLightCubeAlignX;
    targetY=Constants.limeLightCubeALignY;

    finished = false;
    
    
    if (currentR > 30 || currentR <-30) {
      RobotContainer.EndPlaceCommand = true;
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = RobotContainer.s_Limelight.targetX();
    currentY = RobotContainer.s_Limelight.targetY();
    // currentR = RobotContainer.s_Swerve.getPose().getRotation().getDegrees();
    currentR = RobotContainer.s_Swerve.gettempPose().getRotation().getDegrees();


    SmartDashboard.putNumber("Current X", currentX); 
    SmartDashboard.putNumber("Current Y", currentY); 
    SmartDashboard.putNumber("Current R", currentR); 

    errorX = Math.abs(targetX - currentX);
    errorY = Math.abs(targetY - currentY);
    errorR = -(targetR - currentR);

    //Finds shortest path for rotation
    if (errorR > 180){
      errorR = errorR -360;
    } 
    if (errorR < -180){
      errorR = errorR + 360;
    }

    
    calcTranslation = Constants.limeLightCubeAlignP * (targetY - currentY);
    calcStrafe = Constants.limeLightCubeAlignP * (targetX - currentX);
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));
    calcRotation = Constants.autoRotate_P * errorR;

  

    if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
      calcTranslation = Math.min(Constants.limeLightMaxSpeed,calcTranslation);
      //calcTranslation = Math.min(.5, calcTranslation);
    }
  
    if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
      //calcTranslation = Math.min(-.2,calcTranslation);
      calcTranslation = Math.max(-Constants.limeLightMaxSpeed, calcTranslation);
    }

    if (errorY < Constants.limeLightYTolerance){
      calcTranslation = 0;
    }



    if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
      calcStrafe = Math.min(Constants.limeLightMaxSpeed,calcStrafe);
      //calcStrafe = Math.min(.5, calcStrafe);
    }

    if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
      //calcStrafe = Math.min(-.2,calcStrafe);
      calcStrafe = Math.max(-Constants.limeLightMaxSpeed, calcStrafe);
    }
  
    if (errorX < Constants.limeLightXTolerance){
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


    if (calcMagnitude <= Constants.limeLightMinSpeed && calcStrafe != 0) {
      ratio = calcTranslation / calcStrafe;
      calcStrafe = Math.sqrt(Math.pow(Constants.limeLightMinSpeed, 2) / (Math.pow(ratio, 2) + 1)) * Math.signum(calcStrafe);
      calcTranslation = Math.abs(calcStrafe * ratio) * Math.signum(calcTranslation);
  
    }
    if(calcStrafe == 0 && calcTranslation != 0){
      calcTranslation = Math.max(Constants.limeLightMinSpeed,Math.abs(calcTranslation)) * Math.signum(calcTranslation);
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
      calcRotation * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );

  //&& Math.abs(errorR) <= Constants.autoRotateTolerance
   if (errorX <= Constants.limeLightXTolerance && errorY <= Constants.limeLightYTolerance && Math.abs(errorR) <= Constants.autoRotateTolerance){
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