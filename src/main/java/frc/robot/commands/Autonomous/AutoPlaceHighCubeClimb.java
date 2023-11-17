// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.AutoMove;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStation;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceHighCubeClimb extends SequentialCommandGroup {
  /** Creates a new AutoPlaceHighCubeClimb. */
  public AutoPlaceHighCubeClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      //Ejecting start cube
      Commands.parallel(new setIntakeSpeed(Constants.intakeHoldingSpeed),
                        new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS)),
      new WaitCommand(.25),

      // Preparing to climb
      new setLiftPosition(Constants.liftMaxLevelConePOS),
      new setGantryPosition(450),
      new Blank_Command().withTimeout(.05),
      new setGripperPosition(Constants.openGripperPOS),
      new setIntakeSpeed(Constants.intakeRevSpeed),
      new Blank_Command().withTimeout(.5),
      new setGantryPosition(0),
      new setLiftPosition(Constants.liftLowerLevelPOS),
      new DriveOverChargeStation(),
      new GetOnChargeStationFromBack()
    );
  }
}
