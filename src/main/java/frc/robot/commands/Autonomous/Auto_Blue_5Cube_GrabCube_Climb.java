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
import frc.robot.commands.Drive.AutoCubePickup;
import frc.robot.commands.Drive.AutonomousMove;
import frc.robot.commands.Drive.DriveOverChargeStation;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Lift.HomeLiftSpecial;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.RetrieveCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Blue_5Cube_GrabCube_Climb extends SequentialCommandGroup {
  /** Creates a new Auto_Blue_5Cube_GrabCube_Climb. */
  public Auto_Blue_5Cube_GrabCube_Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetFieldOrientation(),
      new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS),
      Commands.parallel(Commands.sequence(new HomeLiftSpecial(), new WaitCommand(.25), new setLiftPosition(Constants.liftMaxLevelCubePOS)),
         new setGantryPosition(Constants.gantryUpperLevelPOS), new setArmPosition(Constants.armExtendedPOS)),
      new setGripperPosition(Constants.openGripperPOS),
      new AdvAutoMove(0, 0, 0, .3, .5, .1, 2, true),
      Commands.parallel(new DropAndRetract(), new DriveOverChargeStation()),
      new AdvAutoMove(4.7, .4, 180, .1, .3, .1, 2, false),
      new AutoCubePickup(4.9, 180, false),
    Commands.parallel(new setGripperPosition(Constants.gripperCubeGrabPOS).withTimeout(Constants.gripperTimeoutCube),
    new DriveForward().withTimeout(.5)), //Test if necessary 
    Commands.parallel(new RetrieveCube(), new AdvAutoMove(4.5, .4, 180, .3, .5, .05, 2, false)),
      new PrepareForClimb(),
      new AutonomousMove(0, 0, 0, true),
      new GetOnChargeStationFromBack()
    );
  }
}
