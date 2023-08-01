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
import frc.robot.commands.Gripper.autoGrip;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPOSConditional;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue_9Cone_HighCube_LowCube extends SequentialCommandGroup {
  /** Creates a new Auto_Blue_9Cone_HighCube_LowCube. */
  public Auto_Blue_9Cone_HighCube_LowCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      new WaitCommand(.25), 
      new setLiftPosition(Constants.liftMaxLevelConePOS),
      new setGantryPosition(Constants.gantryUpperLevelPOS),
      new setIntakeSpeed(Constants.intakeRevSpeed),  
      new Blank_Command().withTimeout(.5),
      new setIntakeSpeed(0),
      new AdvAutoMove(4, .15, 0, .1, .4, .2, 5, true),
      Commands.parallel(new setGantryPosition(Constants.gantryPickPOS), 
              new setLiftPOSConditional(Constants.liftCubeBottomPos)),   
      new AdvAutoMove(4.3, .5, -175, .1, .3, .1, 5, false)
    //   Commands.parallel(new AdvAutoMove(5.2, .5, 180, .3, .3, .05, 5, false),
    //       Commands.sequence( new setIntakeSpeed(Constants.intakeSpeed), new autoGrip(), 
    //       new Blank_Command().withTimeout(.5), new setIntakeSpeed(Constants.intakeHoldingSpeed))),
    //     Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftAutoCubePOS), 
    //       Commands.parallel(new setGantryPosition(Constants.gantryUpperLevelPOS))),
    //            Commands.sequence( new AdvAutoMove(3, .35, -5, .2, .4, .2, 5, false), 
    //                   new AdvAutoMove(1, .35, 0, .1, .4, .1, 5, false),
    //                   new AdvAutoMove(0.15, .7, 0, .1, .4, .1, 5, false))),
    //  // new Blank_Command().withTimeout(.5),
    //   // new setGripperPosition(Constants.openGripperPOS),
    //    //new Blank_Command().withTimeout(.5),
    //    Commands.parallel(Commands.parallel(new setGantryPosition(Constants.gantryRetractedPOS), 
    //    new setLiftPOSConditional(Constants.liftRetrievePOS)), 
    //           Commands.sequence(new AdvAutoMove(4.2, .45, 0, .4, .5, .2, 5, false), 
    //           new AdvAutoMove(4.3, .45, -130, .1, .3, .1, 5, false))),
    // new AdvAutoMove(5.3, 1.45, -135, .2, .4, .1, 5, false)
    // new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
    // Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftLowerLevelPOS), new setArmPosition(Constants.armAutoLowerLevelPOS)),
    // Commands.sequence( new AdvAutoMove(3.5, .6, 0, .2, .4, .2, 5, false), 
    //        new AdvAutoMove(0.4, .4, 0, .5, .8, .2, 5, false)))
   // new setGripperPosition(Constants.openGripperPOS),
   // new AdvAutoMove(3, .45, 0, .2, .4, .2, .5, false)
       
    );
  }
}
