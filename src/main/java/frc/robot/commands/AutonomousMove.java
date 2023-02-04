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
import frc.robot.RobotContainer;

public class AutonomousMove extends CommandBase {
  /** Creates a new AutonomousMove. */
  double targetX;
  double targetY;

  double calcStrafe;
  double calcTranslation;
  double calcRotation;

  double currentX;
  double currentY;

  double errorX;
  double errorY;

  double calcMagnitude; 

  boolean finished;

  public AutonomousMove(double targetX, double targetY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetX = targetX;
    this.targetY = targetY;
    finished = false;
    addRequirements(RobotContainer.s_Swerve);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Swerve.resettempOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    currentX = RobotContainer.s_Swerve.gettempPose().getX();
    currentY = RobotContainer.s_Swerve.gettempPose().getY();

    errorX = Math.abs(targetX - currentX);
    errorY = Math.abs(targetY - currentY);

    calcTranslation = Constants.Swerve.autonomousMove_P * (targetX - currentX);
    calcStrafe = Constants.Swerve.autonomousMove_P * (targetY - currentY);
    calcMagnitude = Math.sqrt(Math.pow(calcTranslation, 2) + Math.pow(calcStrafe, 2));



    if (calcTranslation > 0){ // Set the max and min speed on X coords in positive direction
      calcTranslation = Math.min(Constants.maxSpeedPos,calcTranslation);
    }
  
    if (calcTranslation < 0){ // Set the max and min speed on X coords in negative direction
      calcTranslation = Math.max(-Constants.maxSpeedPos, calcTranslation);
    }
    if (errorX < .05){
      calcTranslation = 0;
    }
    
    

    if (calcStrafe > 0){ // Set the max and min speed on Y coords in positive direction
      calcStrafe = Math.min(Constants.maxSpeedPos,calcStrafe);
    }

    if (calcStrafe < 0){ // Set the max and min speed on Y coords in negative direction
      calcStrafe = Math.max(-Constants.maxSpeedPos, calcStrafe);
    }
    if (errorY < .05){
      calcStrafe = 0;
    }

    if (calcMagnitude <= Constants.minSpeedPos) {
      if (Math.abs(calcTranslation) < Math.abs(calcStrafe)) {
        calcStrafe = Constants.minSpeedPos * Math.signum(calcStrafe);
      }
      if (Math.abs(calcTranslation) > Math.abs(calcStrafe)) {
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
    SmartDashboard.putNumber("Calc Translation",calcTranslation);
    SmartDashboard.putNumber("Calc Strafe",calcStrafe);
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
