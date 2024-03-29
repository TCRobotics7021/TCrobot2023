// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickPlace;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.setLiftPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetrieveCone extends SequentialCommandGroup {
  /** Creates a new RetrieveCone. */
  public RetrieveCone() {
    addCommands(
      new ResetEndPlaceCommand(),
      new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeout),
      new setLiftPosition(Constants.liftRetrievePOS),
      Commands.parallel(new setGantryPosition(Constants.gantryRetractedPOS)),
      new PlaceCommandEnd(),
      new ResetEndPlaceCommand()
    );
  }
}
