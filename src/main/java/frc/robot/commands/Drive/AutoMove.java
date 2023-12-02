// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoMove extends CommandBase {
  double targetX;
  double targetY;
  double targetR;
  
  
  double XYtolerance;
  double Rtolerance;
  double minspeed;
  double maxspeed;
  boolean resetodometry;
  

  double calcStrafe;
  double calcTranslation;
  double calcRotation;

  double currentX;
  double currentY;
  double currentR;
  double previousR;

  double errorX;
  double errorY;
  double errorR;
  double errorM;
  double calcMagnitude; 
  double errorA;

  boolean finished;

  double degrees;

  double ratio;

  Timer clock = new Timer();
  double dt = 0;
  double angularVelocity;
  Timer tonDelay = new Timer();
  double tonTime = 0;

  /** Creates a new AutoMove. */
  public AutoMove(double targetX, double targetY, double targetR, double minspeed, double maxspeed, 
  double XYtolerance, double Rtolerance, boolean resetodometry, double tonTime) {
    this.targetX = targetX;
    this.targetY = targetY;
    this.targetR = targetR;
    this.minspeed = minspeed;
    this.maxspeed = maxspeed;
    this.XYtolerance = XYtolerance;
    this.Rtolerance = Rtolerance;
    this.resetodometry = resetodometry;
    this.tonTime = tonTime;

    finished = false;
    addRequirements(RobotContainer.s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(resetodometry == true){   
    RobotContainer.s_Swerve.resettempOdometry(new Pose2d(0, 0, RobotContainer.s_Swerve.getYaw()));
   }
      clock.reset();
      clock.start();
      tonDelay.reset();
      tonDelay.stop();
      finished = false;
    }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentX = RobotContainer.s_Swerve.gettempPose().getX();
    currentY = RobotContainer.s_Swerve.gettempPose().getY();
    previousR = currentR;
    currentR = RobotContainer.s_Swerve.gettempPose().getRotation().getDegrees();

    dt = clock.get();
    clock.reset();

    angularVelocity = (currentR - previousR)/dt;

    errorX = (targetX - currentX);
    errorY = (targetY - currentY);
    errorM = Math.sqrt(Math.pow(errorX, 2)+ Math.pow(errorY, 2))*Constants.autonomousMove_P;
    if (errorM>maxspeed){
      errorM = maxspeed;
    }
    if (errorM<minspeed && errorM > XYtolerance){

      errorM = minspeed;
    }
    errorA = Math.atan(errorX/errorY);
    calcStrafe = errorM * Math.abs(Math.cos(errorA)) * Math.signum(errorY);
    calcTranslation = errorM * Math.abs(Math.sin(errorA)) * Math.signum(errorX);


    errorR = -(targetR - currentR);


    //finds shortest path for rotation
    if (errorR > 180){
      errorR = errorR -360;
    } 
    if (errorR < -180){
      errorR = errorR + 360;
    }
    // calcRotation = Constants.autoRotate_P * errorR + Constants.autoRotate_D * angularVelocity;
    calcRotation = SmartDashboard.getNumber("autoRotate_P", 0) * errorR + SmartDashboard.getNumber("autoRotate_D", 0) * angularVelocity;
    SmartDashboard.putNumber("angularVelocity", angularVelocity);
    

    if (calcRotation > 0){
      calcRotation = Math.min(SmartDashboard.getNumber("maxAutoRot", Constants.maxAutoRot), calcRotation);
    }
    
    if (calcRotation < 0){
      calcRotation = Math.max(-SmartDashboard.getNumber("maxAutoRot", Constants.maxAutoRot), calcRotation);
    }
  

    if (Math.abs(errorR) > Rtolerance){
      if (calcRotation > 0){
        calcRotation = Math.max(SmartDashboard.getNumber("minAutoRot", Constants.minAutoRot), calcRotation);
      }
      
      if (calcRotation < 0){
        calcRotation = Math.min(-SmartDashboard.getNumber("minAutoRot", Constants.minAutoRot), calcRotation);
      }
    }
    
  //SmartDashboard.putNumber("calcStrafe", calcStrafe);
    // SmartDashboard.putNumber("calcTranslation", calcTranslation);
    // SmartDashboard.putNumber("errorX", errorX);
    // SmartDashboard.putNumber("errorY", errorY);
    // SmartDashboard.putNumber("errorM", errorM);
    // SmartDashboard.putNumber("errorA", errorA);

  RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, calcStrafe).times(Constants.Swerve.maxSpeed), 
      calcRotation * Constants.Swerve.maxAngularVelocity, 
      true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );


  if (Math.abs(errorR) <= Rtolerance && errorM <= XYtolerance) {
    tonDelay.start();

  }
  else {
    tonDelay.reset();
    tonDelay.stop();
  }


  if (tonDelay.get() > tonTime && tonTime > 0){
    finished = true;
  }

  if (Math.abs(errorR) <= Rtolerance && errorM <= XYtolerance && tonTime == 0) {
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
