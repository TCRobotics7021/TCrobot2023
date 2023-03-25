// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceCubeUpperLevel;
import frc.robot.commands.PickPlace.PlaceObjectLowerLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue_5Cube_Overline_Climb extends SequentialCommandGroup {
  /** Creates a new Auto_Blue_5Cube_Overline_Climb. */
  public Auto_Blue_5Cube_Overline_Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelCubePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
      new setGripperPosition(Constants.openGripperPOS),
      Commands.parallel(new DropAndRetract(), new DriveOverChargeStation()),
      new PrepareForClimb(),
      new AutonomousMove(0, 0, 0, true),
      new GetOnChargeStationFromBack()
    );
  }
}
