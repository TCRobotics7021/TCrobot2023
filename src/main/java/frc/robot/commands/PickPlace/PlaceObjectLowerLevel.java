// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickPlace;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Lift.setLiftPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceObjectLowerLevel extends SequentialCommandGroup {
  /** Creates a new PlaceObjectPOS7. */
  public PlaceObjectLowerLevel() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
  new setLiftPosition(Constants.liftLowerLevelPOS), 
  new setGantryPosition(Constants.gantryLowerlevelPOS)
  




    );
  }
}
