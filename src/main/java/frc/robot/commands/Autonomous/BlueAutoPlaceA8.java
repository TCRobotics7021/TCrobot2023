// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.releaseLiftBreak;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueAutoPlaceA8 extends SequentialCommandGroup {
  /** Creates a new BlueAutoPlaceA8. */
  public BlueAutoPlaceA8() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CalibrateLiftAtStartOfMatch(),
      new CalibrateGripperAtStartOfMatch(),
      new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeout),
      new PlaceConeUpperLevel(),
      new releaseLiftBreak().withTimeout(.5),
      new DropAndRetract(),
      new AutonomousMove(1, -.25, 0),
      new AutonomousMove(3.8, 0, 0) 
    
    
    );
  }
}
