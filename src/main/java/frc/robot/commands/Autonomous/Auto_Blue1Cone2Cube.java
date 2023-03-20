// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AutoCubePickup;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLift;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PrepareForPickUp;
import frc.robot.commands.PickPlace.RetrieveCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue1Cone2Cube extends SequentialCommandGroup {
  /** Creates a new PositionTest. */
  public Auto_Blue1Cone2Cube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     new setGripperPosition(Constants.openGripperPOS),
     Commands.parallel(
      Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
            new setLiftPosition(Constants.liftBottomPOS)),
      Commands.sequence( new AdvAutoMove(3, 0, -5, .3, .5, .1, 2, true), new AdvAutoMove(4, -.15, 180, .1, .5, .1, 2, false))),
     
     
     
     
     
    //   Commands.parallel(new setLiftPosition(Constants.liftMidLevelConePOS), 
    //     Commands.sequence(new AdvAutoMove(.25, 0, 5, .3, .5, .1, 5, true),  
    //     new AdvAutoMove(2, 0, 5, .3, .5, .1, 5, false), 
    //     new AdvAutoMove(4, -.52, 180, .1, .5, .1, 2, false))),
    //   // new StartingConePlace(),
    //   // new DropAndRetract(),
    // new setLiftPosition(Constants.liftBottomPOS),
    // new AdvAutoMove(4.8, -.5, 180, .1, .3, .05, 2, false),
    new AutoCubePickup(4.7, 180, false),
    new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
   // Commands.race(new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeout), new AdvAutoMove(4.9, -.52, 180, .1, .3, .03, 2, false)),
    Commands.parallel( new setLiftPosition(Constants.liftMaxLevelCubePOS), 
      Commands.sequence(new AdvAutoMove(3, 0, -5, .3, .5, .1, 5, false), new AdvAutoMove(.7, -.2, 0, .1, .5, .05, 3, false), new AdvAutoMove(.25, -.6, 0, .1, .5, .05, 2, false))),
    Commands.parallel(new setArmPosition(Constants.armExtendedPOS), new setGantryPosition(Constants.gantryUpperLevelPOS)),
    new setGripperPosition(Constants.openGripperPOS),
    Commands.parallel(
    Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
        new setLiftPosition(Constants.liftBottomPOS)),
        Commands.sequence( new AdvAutoMove(5, 0, -5, .3, .5, .1, 5, false), new AdvAutoMove(5, 0, 90, .1, .5, .1, 2, false)))


      // new PrepareForPickUp(),
      // new RetrieveCube(),
      // new AutonomousMove(3, -.25, 0, false)
    );
  }
}
