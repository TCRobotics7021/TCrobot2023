// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.HomeAll;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Place_Sit extends SequentialCommandGroup {
  /** Creates a new Place_Sit. */
  public Place_Sit() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

      addCommands(
        new ResetFieldOrientation(),
        Commands.parallel(new setIntakeSpeed(Constants.intakeSpeed),
        new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS)),
        new WaitCommand(.25), 
        Commands.parallel(new setLiftPosition(Constants.liftMaxLevelConePOS), new setGantryPosition(200)),
        new setGantryPosition(450),
        new setIntakeSpeed(Constants.intakeRevSpeed),  
        new Blank_Command().withTimeout(.05),
        new setIntakeSpeed(0),
        new HomeAll()
    );
  }
}
