// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Autonomous.CalibrateGripperAtStartOfMatch;
import frc.robot.commands.Autonomous.CalibrateLiftAtStartOfMatch;
import frc.robot.commands.Autonomous.StartingConePlace;
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
public class Auto_Blue9Cone8Cube extends SequentialCommandGroup {
  /** Creates a new Auto_Blue9Cone8Cube. */
  public Auto_Blue9Cone8Cube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     new setGripperPosition(Constants.openGripperPOS),
     Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS), new AdvAutoMove(4, .3, 0, .3, .5, .1, 5, true)),
    // Commands.parallel(new setLiftPosition(Constants.liftMidLevelCubePOS), new AdvAutoMove(3, .3, 0, .2, .4, .1, 5, false)), 
       // Commands.sequence(new AdvAutoMove(4, .3, 0, .2, .4, .1, 5, true),  
       // new AdvAutoMove(3, .3, 180, .2, .4, .1, 5, false), 
    Commands.parallel(new AdvAutoMove(4.2, .5, -170, .3, .5, .1, 2, false), new setLiftPosition(Constants.liftBottomPOS)),
      // new StartingConePlace(),
      // new DropAndRetract()1
    // new AdvAutoMove(4.8, -.5, 180, .1, .3, .05, 2, false),
    new AutoCubePickup(4.9, 180, false),
    new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
   // Commands.race(new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeout), new AdvAutoMove(4.9, -.52, 180, .1, .3, .03, 2, false)),
    Commands.parallel( new setLiftPosition(Constants.liftMaxLevelCubePOS), 
    Commands.sequence(new AdvAutoMove(4, .55, 0, .3, .5, .1, 2, isFinished()), new AdvAutoMove(1, .55, 0, .2, .4, .1, 2, false), new AdvAutoMove(.3, .9, 0, .2, .5, .05, 2, false))),
    Commands.parallel(new setArmPosition(Constants.armExtendedPOS), new setGantryPosition(Constants.gantryUpperLevelPOS)),
    new setGripperPosition(Constants.openGripperPOS),
    new setArmPosition(Constants.armRetractedPOS)

    );
  }
}

