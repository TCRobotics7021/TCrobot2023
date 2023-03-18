// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.releaseLiftBreak;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCubeUpper extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCubeUpper. */
  public AutoPlaceCubeUpper() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new CalibrateLiftAtStartOfMatch(),
      new CalibrateGripperAtStartOfMatch(Constants.GripperStartingcubePOS),
      new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeout),
      new PlaceConeUpperLevel(),
      new releaseLiftBreak().withTimeout(.5),
      new DropAndRetract()
    );
  }
}
