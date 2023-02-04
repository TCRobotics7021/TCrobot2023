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

public class Rotation extends CommandBase {
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

  public Rotation(double targetX, double targetY,double targetR) {

    this.targetR=targetR;
    finished = false;
    addRequirements(RobotContainer.s_Swerve);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Swerve.resettempOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }


  // Called every time the scheduler runs while the command is scheduled.
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  @Override
  public void execute() {

    currentR = RobotContainer.s_Swerve.gettempPose().getRotation().getDegrees();

        
  
    errorR = Math.abs(targetR - currentR);
    

    calcRotation = -Constants.Swerve.autonomousRot_P * (targetR - currentR);



    if (calcRotation > 0){ // Set the max and min speed on X coords in positive direction
      calcRotation = Math.min(Constants.maxSpeedRot,calcRotation);
      calcRotation= Math.max(.2,calcRotation);
     }
  
     if (calcRotation < 0){ // Set the max and min speed on X coords in negative direction
      calcRotation = Math.max(-Constants.maxSpeedRot, calcRotation);
      calcRotation = Math.min(-.2,calcRotation);
     }

     RobotContainer.s_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      calcRotation * Constants.Swerve.maxAngularVelocity, 
      false, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true);
      if (errorR <= 2 ){
        finished = true;
      }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.drive(
      new Rotation2d(0, 0).times(Constants.Swerve.maxSpeed), 
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
