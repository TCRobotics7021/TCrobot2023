// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GetOnChargeStation extends CommandBase {
  /** Creates a new GetOnChargeStation. */

  int state = 0;
  boolean Finished = false;
  Timer balanceTimer = new Timer();
  double currentAngle = 0;
  double calcTranslation = 0;

  public GetOnChargeStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    Finished = false;
    currentAngle = RobotContainer.s_Swerve.GetPitch();
    balanceTimer.reset();
    balanceTimer.start();
    calcTranslation = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = RobotContainer.s_Swerve.GetPitch();
    if (Math.abs(currentAngle) > Constants.balanceAngle ) {
      balanceTimer.reset();
    }


    if (state == 0 && Constants.climbStartedAngle < currentAngle) {
      state = 1;
    }

    if (state == 1 && currentAngle < Constants.startedTiltDownAngle) {
      state = 2;
    }

    if (state == 2) {
      if (currentAngle < -Constants.balanceAngle) {
        state = 3;
      }
      if (balanceTimer.get() > Constants.balanceTime) {
        Finished = true;
      }
    }
    if (state == 3) {
      if (Constants.balanceAngle < currentAngle) {
        state = 4;
      }
      if (balanceTimer.get() > Constants.balanceTime) {
        Finished = true;
      }
    }
    if (state == 4) {
      
      if (balanceTimer.get() > Constants.balanceTime) {
        Finished = true;
      }
      if (currentAngle < -Constants.balanceAngle) {
        state = 3;
      }
    }
    if(state == 0){
      calcTranslation = Constants.climbState0_StartingSpeed;
    }
    if (state == 1) {
      calcTranslation = Constants.climbState1_ClimbingSpeed;
    }
    if (state ==3) {
      if (Math.abs(currentAngle) < Constants.balanceAngle) {
        calcTranslation = 0;
      } else {
        calcTranslation = Constants.climbState3_REVspeed;
      }
    }
    if (state ==4) {
      if (Math.abs(currentAngle) < Constants.balanceAngle) {
        calcTranslation = 0;
      } else {
        calcTranslation = Constants.climbState4_FWDspeed;
      }
    }

    RobotContainer.s_Swerve.drive(
      new Translation2d(calcTranslation, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      true, //Fieldcentric - !robotCentricSup.getAsBoolean(), 
      true
  );
  // SmartDashboard.putNumber("state", state);
  //   SmartDashboard.putNumber("output", calcTranslation);
  //   SmartDashboard.putNumber("Balanced Time", balanceTimer.get());
  //   SmartDashboard.putBoolean("Finished", Finished);
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
    state = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Finished;
  }
}
