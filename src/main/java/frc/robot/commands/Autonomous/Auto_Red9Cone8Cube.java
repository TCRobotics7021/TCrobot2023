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
import frc.robot.commands.Lift.setLiftPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Red9Cone8Cube extends SequentialCommandGroup {
  /** Creates a new Auto_Red1Cone2Cube. */
  public Auto_Red9Cone8Cube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     new setGripperPosition(Constants.openGripperPOS),
     Commands.parallel(
      Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
            new setLiftPosition(Constants.liftBottomPOS)),
      Commands.sequence( new AdvAutoMove(3, .15, 5, .3, .7, .1, 2, true), new AdvAutoMove(4, .15, 180, .1, .6, .1, 2, false))),
     
     
    new AutoCubePickup(4.7, 180, false),
    new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
    Commands.parallel( new setLiftPosition(Constants.liftMaxLevelCubePOS), 
      Commands.sequence(new AdvAutoMove(3, .15, -5, .3, .7, .1, 5, false), new AdvAutoMove(.7, .2, 0, .1, .6, .05, 4, false), new AdvAutoMove(.25, .6, 0, .1, .5, .05, 2, false))),
    Commands.parallel(new setArmPosition(Constants.armExtendedPOS), new setGantryPosition(Constants.gantryUpperLevelPOS)),
    new setGripperPosition(Constants.openGripperPOS),
    Commands.parallel(
    Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
        new setLiftPosition(Constants.liftBottomPOS)),
        Commands.sequence( new AdvAutoMove(5.5, 0, 5, .3, .7, .1, 5, false), new AdvAutoMove(5.5, 0, -90, .1, .5, .1, 2, false)))
    );
  }
}
