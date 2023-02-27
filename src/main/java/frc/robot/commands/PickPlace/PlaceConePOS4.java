// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickPlace;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.MoveToPosReletiveToTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConePOS4 extends SequentialCommandGroup {
  /** Creates a new PlaceCubePOS2. */
  public PlaceConePOS4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ResetEndPlaceCommand(),
    // new MoveToPosReletiveToTarget(1.25, 0, 0),
    // new WaitCommand(0.5),
    // new MoveToPosReletiveToTarget(0.56,-0.75 , 0),
    new PlaceCommandStart(),
    new setLiftPosition(Constants.liftMidLevelConePOS),
    Commands.parallel(new setArmPosition(Constants.armMidLevelPOS), new setGantryPosition(Constants.gantryUpperLevelPOS)),
     new ResetEndPlaceCommand()



    );
  }
}
