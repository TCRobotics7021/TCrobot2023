// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStation;

import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.RetrieveCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Red4Cone_GrabCone_Climb extends SequentialCommandGroup {
  /** Creates a new Auto_RedTest5. */
  public Auto_Red4Cone_GrabCone_Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      new AdvAutoMove(0, 0, 0, .5, .7, .1, 1, true),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelConePOS)),
      new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
  new setLiftPosition(Constants.liftMaxLevelConeDip),
  new setGripperPosition(Constants.openGripperPOS),
      Commands.parallel(Commands.sequence(new DropAndRetract(), new setGantryPosition(Constants.gantryPickPOS)), new DriveOverChargeStation()),
    Commands.parallel(new AdvAutoMove(4.4, 0, 180, .4, .6, .1, 2, false),
      Commands.sequence(Commands.parallel(new setArmPosition(Constants.armPickPOS), new setGantryPosition(Constants.gantryPickPOS)), 
            new setLiftPosition(Constants.liftBottomPOS))),
      new AdvAutoMove(4.9, 0, 180, .1, .3, .1, 2, false),
      new setGripperPosition(Constants.gripperConeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
      //new PrepareForClimb(),
      Commands.parallel(new RetrieveCone(),
      new AutonomousMove(0, 0, 0, true)),
      new GetOnChargeStationFromBack()
    );
  }
}
