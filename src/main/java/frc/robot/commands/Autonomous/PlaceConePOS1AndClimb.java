// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutonomousMove;
import frc.robot.commands.Drive.GetOnChargeStation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.PlaceConePOS4;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConePOS1AndClimb extends SequentialCommandGroup {
  /** Creates a new PlaceConePOS1AndClimb. */
  public PlaceConePOS1AndClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new setLiftPosition(Constants.liftAutoStartPOS), 
    new setGripperPosition(Constants.gripperConeGrabPOS),
    new PlaceConePOS4(),
    new DropAndRetract(),
    new AutonomousMove(0.5, 0, 0),
    new setGantryPosition(Constants.gantryClimbPOS),
    Commands.parallel(new AutonomousMove(0, 0, 180), new setLiftPosition(Constants.liftClimbPOS)),
    new AutonomousMove(-1, 0, 0),
    new GetOnChargeStation()







    );
  }
}
