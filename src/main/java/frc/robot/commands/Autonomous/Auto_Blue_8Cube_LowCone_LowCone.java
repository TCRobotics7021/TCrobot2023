// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.setLiftPOSConditional;
import frc.robot.commands.Lift.setLiftPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue_8Cube_LowCone_LowCone extends SequentialCommandGroup {
  /** Creates a new Auto_Blue_8Cube_GrabCone_GrabCone. */
  public Auto_Blue_8Cube_LowCone_LowCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(850), //change
       Commands.parallel(new setLiftPosition(Constants.liftAutoCubePOS), //leave for now
          new setGantryPosition(Constants.gantryAutoUpperLevelPOS), new setArmPosition(Constants.armAutoExtendedPOS)),
       new setGripperPosition(Constants.openGripperPOS),
       Commands.parallel(
         Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
               new setLiftPOSConditional(Constants.liftBottomPOS)),
         Commands.sequence( new AdvAutoMove(3.2, -.5, 0, .5, .5, .2, 5, true), new AdvAutoMove(4.3, 0.1, -175, .1, .3, .1, 5, false))),
         new AdvAutoMove(4.9, -.05, 180, .3, .3, .1, 5, false),
         new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
         Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftLowerLevelPOS), new setArmPosition(Constants.armAutoLowerLevelPOS)),
                Commands.sequence( new AdvAutoMove(4.3, -.1, 0, .2, .4, .2, 5, false), 
                       new AdvAutoMove(.15, 0, 0, .2, .5, .1, 5, false))),
         new setGripperPosition(Constants.openGripperPOS),
        Commands.parallel(new setArmPosition(Constants.armPickPOS), 
               Commands.sequence( new AdvAutoMove(3.6, 0, 0, .5, .5, .2, 5, false), 
               new AdvAutoMove(4.4, 0.1, -130, .1, .3, .1, 5, false))),
       Commands.parallel(new setLiftPosition(Constants.liftBottomPOS), new AdvAutoMove(5.05, .95, -135, .2, .3, .1, 5, false)),
     new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
     Commands.parallel(Commands.sequence(new setLiftPosition(Constants.liftLowerLevelPOS), new setArmPosition(Constants.armAutoLowerLevelPOS)),
     Commands.sequence( new AdvAutoMove(3.5, -.1, 0, .1, .3, .2, 5, false), 
            new AdvAutoMove(0, -.15, 0, .2, .5, .1, 5, false))),
     new setGripperPosition(Constants.openGripperPOS),
     new AdvAutoMove(3, -.1, 5, .2, .4, .2, .5, false)
    );
  }
}
