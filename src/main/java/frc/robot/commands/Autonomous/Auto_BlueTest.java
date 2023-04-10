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
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.PlaceCubeUpperLevel;
import frc.robot.commands.PickPlace.RetrieveCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_BlueTest extends SequentialCommandGroup {
  /** Creates a new Auto_BlueTest. */
  public Auto_BlueTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
     new setLiftPosition(Constants.liftMaxLevelConeDip),
     new setGripperPosition(Constants.openGripperPOS),
     Commands.parallel(
      Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
            new setLiftPosition(Constants.liftBottomPOS)),
      Commands.sequence( new AdvAutoMove(3, 0, -5, .3, .5, .1, 2, true), new AdvAutoMove(4, -.55, 180, .3, .5, .1, 2, false),  new AdvAutoMove(4.5, -.55, 180, .1, .2, .05, 2, false))),
     Commands.parallel(new AdvAutoMove(4.8, -.55, 180, .1, .2, .01, 2, false), new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTestTimeout)),
      Commands.parallel(
        Commands.sequence(new RetrieveCone(),new PlaceCubeUpperLevel()),
         Commands.sequence(new AdvAutoMove(4.5, -.1, 0, .3, .5, .05, 5, false), new AdvAutoMove(.3, -.4, 0, .3, .5, .05, 5, false), new AdvAutoMove(.3, -.4, 5, .3, .5, .05, 1, false))),
    new setGripperPosition(Constants.openGripperPOS),
    Commands.parallel(Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
            new setLiftPosition(Constants.liftBottomPOS)),
    new AdvAutoMove(4.5, -.1, 120, .3, .5, .05, 4, false)),
  new AdvAutoMove(4.5, -.1, 120, .3, .5, .05, 2, false)
    );
  }
}
