// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Old;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Autonomous.CalibrateLiftAtStartOfMatch;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AutoMove;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.HomeGripper;
import frc.robot.commands.Gripper.autoGrip;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Red_BumpSide_OFF extends SequentialCommandGroup {
  /** Creates a new Test_Blue_9Cone_HighCone_MidCone. */
  public Auto_Red_BumpSide_OFF() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      Commands.parallel(new setIntakeSpeed(Constants.intakeSpeed),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS)),
      new WaitCommand(.25), 
      new setLiftPosition(Constants.liftMaxLevelConePOS),
      new setGantryPosition(450),
      new setGripperPosition(Constants.openGripperPOS),
      new setIntakeSpeed(Constants.intakeRevSpeed),  
      new Blank_Command().withTimeout(.05),
     
      new setIntakeSpeed(0),
 Commands.parallel(
          new setGantryPosition(400), 
          new setLiftPosition(400), 
            Commands.sequence(new AutoMove(.5, .6, 0, .1, .2, .2, 10, true, 0), 
                            new AutoMove(2.5, .7, 90, .1, .4, .4, 20, false, 0))),

   Commands.parallel(new AutoMove(4, .3, 180, .1, .4, .1, 4, false, .1), Commands.sequence(new setGantryPosition(Constants.gantryPickPOS), 
                    new setLiftPosition(Constants.liftBottomPOS))),

     Commands.parallel(new AutoMove(5.1, .3, 180, .2, .4, .1, 2, false, 0),
     Commands.sequence(new setIntakeSpeed(Constants.intakeSpeed), new autoGrip())),  

   new Blank_Command().withTimeout(.05), new setIntakeSpeed(Constants.intakeHoldingSpeed), 
  // Commands.parallel(
     Commands.sequence(new setLiftPosition(Constants.liftMidLevelCubePOS), 
     new setGantryPosition(Constants.gantryMidLevelPOS))
  //  new AutoMove(1.5, .3, 1, .2, .4, .3, 6, false, 0)),
  //  Commands.parallel(new AutoMove(.35, 0, 0, .2, .4, .05, 1, false, 0)), 
  // //  new setLiftPosition(Constants.liftMaxLevelCubePOS)),
  // new setGripperPosition(Constants.openGripperPOS),
  //  new setIntakeSpeed(Constants.intakeRevSpeed),  
  //     new Blank_Command().withTimeout(.05),
  //  new setIntakeSpeed(0),
  //   Commands.parallel(new AutoMove(1.5, .7, 0, .1, .2, .2, 10, false, 0), new setGantryPosition(300), new setLiftPosition(750)),
  //   Commands.parallel(new AutoMove(3.7, .55, 135, .3, .5, .1, 3, false, 0), new HomeGripper(),
  //            new setLiftPosition(Constants.liftRetrievePOS), new setGantryPosition(Constants.gantryPickPOS)),
  //   new AutoMove(4.5, 0, 135, .2, .4, .1, 3, false, 0) 
  //  new setIntakeSpeed(Constants.intakeSpeed), new autoGrip()), new setLiftPosition(Constants.liftBottomPOS)),  
  //   new Blank_Command().withTimeout(.05), new setIntakeSpeed(Constants.intakeHoldingSpeed
  //   Commands.parallel(
  //     Commands.sequence(new setLiftPosition(Constants.liftMidLevelCubePOS), new setGantryPosition(Constants.gantryMidLevelPOS)),
  //     new AutoMove(4, 0, 0, .2, .5, .3, 5, false, 0)),
  //     new AutoMove(1.2, -0, 0, .1, .6, .05, 1, false, 0),
  //     Commands.parallel(new AutoMove(.6, 0, 0, .1, .4, .05, 2, false, 0), new setLiftPosition(Constants.liftMaxLevelCubePOS))
    );
  }
}
