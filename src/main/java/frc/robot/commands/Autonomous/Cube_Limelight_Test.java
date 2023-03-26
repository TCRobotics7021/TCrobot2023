// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutoCubePickup;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gripper.setGripperPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cube_Limelight_Test extends SequentialCommandGroup {
  /** Creates a new Cube_Limelight_Test. */
  public Cube_Limelight_Test() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new AutonomousMove(0, 0, -175, true),
      new AutoCubePickup(.5, 180, false),
      Commands.parallel(new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
      new DriveForward().withTimeout(1))
    );
  }
}
