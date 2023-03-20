// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;
import frc.robot.commands.PickPlace.PlaceCubeUpperLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCubeB8OverLineAndClimb extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCubeB8OverLineAndClimb. */
  public AutoPlaceCubeB8OverLineAndClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlaceCubeUpperLevel(),
      new setGripperPosition(Constants.openGripperPOS),
      Commands.parallel(new DropAndRetract(), new DriveOverChargeStation()),
      new PrepareForClimb(),
      new AutonomousMove(0, 0, 0, true),
      new GetOnChargeStationFromBack()
      
    );
  }
}
