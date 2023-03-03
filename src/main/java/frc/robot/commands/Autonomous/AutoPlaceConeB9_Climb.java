// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.ClimbOnly;
import frc.robot.commands.Drive.GetOnChargeStation;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.releaseLiftBreak;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceCommandEnd;
import frc.robot.commands.PickPlace.PlaceConeMidLevel;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceConeB9_Climb extends SequentialCommandGroup {
  /** Creates a new AutoPlaceConeB9_Climb. */
  public AutoPlaceConeB9_Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CalibrateLiftAtStartOfMatch(),
    new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
    new PlaceConeUpperLevel(),
    new AutonomousMove(-.25, 0, 0),
    new releaseLiftBreak().withTimeout(.5),
    new setGripperPosition(Constants.openGripperPOS),
    Commands.parallel(new AutonomousMove(.25, .25, 0), new setGantryPosition(Constants.gantryRetractedPOS), new setArmPosition(Constants.armRetractedPOS)),
    new PlaceCommandEnd(),
    Commands.parallel(new AutonomousMove(0, 0, 180), new PrepareForClimb()),
    new GetOnChargeStation()
    );
  }
}
