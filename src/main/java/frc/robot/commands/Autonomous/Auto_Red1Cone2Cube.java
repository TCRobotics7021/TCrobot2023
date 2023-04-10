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
public class Auto_Red1Cone2Cube extends SequentialCommandGroup {
  /** Creates a new Auto_Red9Cone8Cube. */
  public Auto_Red1Cone2Cube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     new setGripperPosition(Constants.openGripperPOS),
     Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS), new AdvAutoMove(4, -.3, 0, .3, .5, .1, 5, true)),
    Commands.parallel(new AdvAutoMove(4.2, -.5, 170, .3, .5, .1, 2, false), new setLiftPosition(Constants.liftBottomPOS)),
    new AutoCubePickup(4.9, 180, false),
    Commands.parallel(new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
    new DriveForward().withTimeout(.5)),
    Commands.parallel( new setLiftPosition(Constants.liftMaxLevelCubePOS), 
    Commands.sequence(new AdvAutoMove(4, -.5, 0, .3, .5, .1, 2, isFinished()), new AdvAutoMove(1, -.5, 0, .2, .4, .1, 2, false), new AdvAutoMove(.3, -.9, 0, .2, .5, .05, 2, false))),
    Commands.parallel(new setArmPosition(Constants.armExtendedPOS), new setGantryPosition(Constants.gantryUpperLevelPOS)),
    new setGripperPosition(Constants.openGripperPOS),
    new setArmPosition(Constants.armRetractedPOS)
    );
  }
}
