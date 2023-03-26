// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limeLight_subsystem;

public class AutoCubePickup extends CommandBase {
  /** Creates a new AutoCubePickup. */
  double targetX;
  double targetY;
  double targetR;
  boolean resetodometry;

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

  public AutoCubePickup(double targetX, double targetR, boolean resetodometry) {

    this.targetX = targetX;
    this.targetR = targetR;
    this.resetodometry = resetodometry;
    finished = false;
    addRequirements(RobotContainer.s_Swerve, RobotContainer.s_Limelight);
    


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Limelight.setPipeline(0);
    if(resetodometry == true){
      RobotContainer.s_Swerve.resettempOdometry(new Pose2d(0, 0, RobotContainer.s_Swerve.getYaw()));
    }
     finished = false;
     
     targetY =-13;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentX = RobotContainer.s_Swerve.gettempPose().getX();
    currentY = RobotContainer.s_Limelight.targetX();
    currentR = RobotContainer.s_Swerve.gettempPose().getRotation().getDegrees();

    errorX = Math.abs(targetX - currentX);
    errorY = Math.abs(targetY - currentY);
    errorR = -(targetR - currentR);

    //finds shortest path for rotation
    if (errorR > 180){
      errorR = errorR -360;
    } 
    if (errorR < -180){
      errorR = errorR + 360;
    }

    calcTranslation = Constants.autonomousMove_P * (targetX - currentX);
    calcStrafe = Constants.limeLightCubeAlignP * (targetY - currentY);
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));
    calcRotation = Constants.autoRotate_P * errorR;

    if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
      calcTranslation = Math.min(Constants.limeLightMaxSpeed,calcTranslation);
    }
  
    if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
      calcTranslation = Math.max(-Constants.limeLightMaxSpeed, calcTranslation);
    }
    if (errorX < Constants.errorTolerance){
      calcTranslation = 0;
    }
    
    

    if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
      calcStrafe = Math.min(Constants.limeLightMaxSpeed,calcStrafe);
    }

    if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
      calcStrafe = Math.max(-Constants.limeLightMaxSpeed, calcStrafe);
    }
    if (errorY < Constants.limeLightXTolerance){
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

    if (Math.abs(errorR) <= Constants.autoRotateTolerance){
        RobotContainer.s_Swerve.drive(
          new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
          calcRotation * Constants.Swerve.maxAngularVelocity, 
          true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
          true
      );
    }else{
      RobotContainer.s_Swerve.drive(
          new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
          calcRotation * Constants.Swerve.maxAngularVelocity, 
          true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
          true
      );
    }

    if (errorX <= Constants.errorTolerance && Math.abs(errorR) <= Constants.autoRotateTolerance){
      finished = true;
    }
      SmartDashboard.putNumber("Target X", targetX);
      SmartDashboard.putNumber("Current X", currentX);
      SmartDashboard.putNumber("Calc Translation",calcTranslation);
      SmartDashboard.putNumber("Calc Strafe",calcStrafe);
      SmartDashboard.putNumber("Current Y", currentY);
    // SmartDashboard.putNumber("Calc Rotate", calcRotation);

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
