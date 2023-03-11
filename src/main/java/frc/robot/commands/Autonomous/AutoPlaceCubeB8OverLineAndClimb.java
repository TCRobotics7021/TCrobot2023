// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.PickPlace.DropAndRetract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCubeB8OverLineAndClimb extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCubeB8OverLineAndClimb. */
  public AutoPlaceCubeB8OverLineAndClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DropAndRetract(),
      new DriveOverChargeStation(),
      new PrepareForClimb(),
      new GetOnChargeStationFromBack()
    );
  }
}
