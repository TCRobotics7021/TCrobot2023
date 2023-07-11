// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AutoCubePickup;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPOSConditional;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;
import frc.robot.commands.PickPlace.RetrieveCone;
import frc.robot.commands.PickPlace.RetrieveCube;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue_1Cone_HighCube_LowCone extends SequentialCommandGroup {
  /** Creates a new Auto_Blue_1Cone_GrabCone_GrabCube. */
  public Auto_Blue_1Cone_HighCube_LowCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     Commands.parallel(
      new setGripperPosition(Constants.openGripperPOS),
       Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS), 
              new setLiftPOSConditional(Constants.liftBottomPOS)),
        Commands.sequence( new AdvAutoMove(2.8, -.15, -5, .5, .5, .2, 5, true),
          new AdvAutoMove(4.3, -.4, -175, .1, .3, .1, 5, false))),
        Commands.parallel(new AdvAutoMove(4.9, -.4, 180, .3, .3, .1, 5, false),
          Commands.sequence(new Blank_Command().withTimeout(Constants.GripperCubeDelay), new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeout))),
        Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftAutoCubePOS), 
          Commands.parallel(new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS))),
               Commands.sequence( new AdvAutoMove(3, -.2, -5, .2, .4, .2, 5, false), 
                      new AdvAutoMove(0.3, -.6, 0, .2, .5, .1, 5, false))),
        new setGripperPosition(Constants.openGripperPOS),
       Commands.parallel(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS), 
       new setLiftPOSConditional(Constants.liftBottomPOS)), 
              Commands.sequence(new AdvAutoMove(3.5, -.15, 0, .5, .5, .2, 5, false), 
              new AdvAutoMove(4, 0, 130, .1, .3, .1, 5, false))),
    new AdvAutoMove(5, -1.15, 135, .2, .4, .1, 5, false)
    // new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
    // Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftLowerLevelPOS), new setArmPosition(Constants.armAutoLowerLevelPOS)),
    // Commands.sequence( new AdvAutoMove(3.5, -.6, 0, .2, .4, .2, 5, false), 
    //        new AdvAutoMove(0.4, -.4, 0, .5, .8, .2, 5, false))),
    // new setGripperPosition(Constants.openGripperPOS),
    // new AdvAutoMove(3, -.45, 0, .2, .4, .2, .5, false)
       
      );
  }
}
