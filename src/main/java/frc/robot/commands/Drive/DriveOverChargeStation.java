// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveOverChargeStation extends CommandBase {
  /** Creates a new DriveOverChargeStation. */
  int state = 0;
  boolean Finished = false;
  Timer LevelTimer = new Timer();
  double currentAngle = 0;
  double calcTranslation = 0;
  public DriveOverChargeStation() {
   addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    Finished = false;
    currentAngle = RobotContainer.s_Swerve.GetPitch();
    LevelTimer.reset();
    LevelTimer.start();
    calcTranslation = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = RobotContainer.s_Swerve.GetPitch();
    if (Math.abs(currentAngle) > Constants.balanceAngle ) {
      LevelTimer.reset();
    }


    if (state == 0 && Constants.climbStartedAngle < currentAngle) {
      state = 1;
       }
  
       if (state==1 && currentAngle < 0){
       state = 2;
      }

     
   if (state==2 && LevelTimer.get() > Constants.driveOverDelay){
      Finished = true;
      }
if (state==0 ){
  calcTranslation = Constants.driveOverState0_StartingSpeed;
}

if (state==1){
  calcTranslation = Constants.driveOverState1_ClimbingSpeed;
}

if (state==2){
  calcTranslation = Constants.driveOverState2_DescendingSpeed;
}
RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
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
    return Finished;
  }
}
