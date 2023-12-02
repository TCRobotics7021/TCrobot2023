// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickPlace;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.autoGrip;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;
import frc.robot.subsystems.Gantry;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareForPickUp extends SequentialCommandGroup {
  /** Creates a new GrabCone. */
  public PrepareForPickUp() {
    // Add your coddSeqmmands in the addCommands() call, e.g.
  
  
   addCommands( 
    new ResetEndPlaceCommand(),
    Commands.parallel(new setGantryPosition(Constants.gantryPickPOS), new setGripperPosition(Constants.openGripperPOS)),
    Commands.sequence(new setLiftPosition(Constants.liftBottomPOS), 
    new setIntakeSpeed(Constants.intakeSpeed), new autoGrip(), 
    new Blank_Command().withTimeout(.5), new setIntakeSpeed(Constants.intakeHoldingSpeed), 
    new setLiftPosition(Constants.liftRetrievePOS),
    Commands.parallel(new setGantryPosition(Constants.gantryRetractedPOS)),
    new PlaceCommandEnd()),
    new ResetEndPlaceCommand()
   );

  }


}

