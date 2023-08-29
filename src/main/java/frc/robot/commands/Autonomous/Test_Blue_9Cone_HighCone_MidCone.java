// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.AdvAutoMove;
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
public class Test_Blue_9Cone_HighCone_MidCone extends SequentialCommandGroup {
  /** Creates a new Test_Blue_9Cone_HighCone_MidCone. */
  public Test_Blue_9Cone_HighCone_MidCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      new WaitCommand(.25), 
      Commands.parallel(new setLiftPosition(Constants.liftMaxLevelConePOS), new setGantryPosition(300)),
      new setGantryPosition(Constants.gantryUpperLevelPOS),
      new setIntakeSpeed(Constants.intakeRevSpeed),  
      new Blank_Command().withTimeout(.05),
      new setIntakeSpeed(0),
  Commands.sequence(new AdvAutoMove(.5, 0, 0, .2, .4, .2, 10, true), 
  new AdvAutoMove(1.5, 0, 30, .2, .4, .6, 5, false)),
    Commands.parallel(new setLiftPosition(Constants.liftRetrievePOS), 
     new setGantryPosition(Constants.gantryPickPOS),
      new AdvAutoMove(4, 0, 175, .2, .4, .3, 5, false)),
     Commands.parallel(new AdvAutoMove(5.4, -.3, 180, .2, .4, .1, 2, false), Commands.sequence(new setIntakeSpeed(Constants.intakeSpeed), new autoGrip()), new setLiftPosition(Constants.liftBottomPOS)),  
     new Blank_Command().withTimeout(.05), new setIntakeSpeed(Constants.intakeHoldingSpeed), 
     Commands.parallel(
    Commands.sequence(new setLiftPosition(Constants.liftMidLevelCubePOS), new setGantryPosition(Constants.gantryMidLevelPOS)),
    new AdvAutoMove(1.5, -.4, -1, .2, .4, .3, 6, false)),
    Commands.parallel(new AdvAutoMove(.6, -.8, 1, .2, .4, .05, 1, false), 
    new setLiftPosition(Constants.liftMaxLevelCubePOS)),
    new setIntakeSpeed(Constants.intakeRevSpeed),  
      new Blank_Command().withTimeout(.05),
   new setIntakeSpeed(0),
    new AdvAutoMove(1, -.1, 0, .2, .4, .6, 10, false),
    Commands.parallel(new AdvAutoMove(5, -.2, 90, .5, .5, .6, 10, false), new HomeGripper(),
             Commands.sequence(new setLiftPosition(Constants.liftRetrievePOS), 
            new setGantryPosition(Constants.gantryPickPOS))),
            new AdvAutoMove(6, -.1, 90, .2, .4, .3, 5, false),
    Commands.parallel(new AdvAutoMove(6, -1, 90, .2, .4, .1, 5, false), Commands.sequence(new setIntakeSpeed(Constants.intakeSpeed), new autoGrip()), new setLiftPosition(Constants.liftBottomPOS)),  
    new Blank_Command().withTimeout(.05), new setIntakeSpeed(Constants.intakeHoldingSpeed),
    Commands.parallel(
      Commands.sequence(new setLiftPosition(Constants.liftMidLevelCubePOS), new setGantryPosition(Constants.gantryMidLevelPOS)),
      new AdvAutoMove(4, 0, -1, .2, .4, .3, 5, false)),
      new AdvAutoMove(1.2, -.2, 1, .2, .4, .05, 1, false)
    //   Commands.parallel(new AdvAutoMove(.6, -.7, 0, .1, .4, .05, 2, false), new setLiftPosition(Constants.liftMaxLevelCubePOS))
    );
  }
}
