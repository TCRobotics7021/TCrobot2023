// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickPlace;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.autoDoubleSubGrip;
import frc.robot.commands.Gripper.autoGrip;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareForSubPickup extends SequentialCommandGroup {
  /** Creates a new PrepareForSubPickup. */
  public PrepareForSubPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEndPlaceCommand(),
      //add limelight
      Commands.parallel(new setLiftPosition(Constants.liftSubstationPOS), new setGripperPosition(Constants.gripperSubstationPickupPOS)),
      new setGantryPosition(Constants.gantrySubPOS), 
      new setIntakeSpeed(Constants.intakeSpeed), new autoDoubleSubGrip(), 
      new Blank_Command().withTimeout(.5), new setIntakeSpeed(Constants.intakeHoldingSpeed), 
      new setLiftPosition(1275),
      new PlaceCommandEnd(),
      new ResetEndPlaceCommand()


    );
  }
}
