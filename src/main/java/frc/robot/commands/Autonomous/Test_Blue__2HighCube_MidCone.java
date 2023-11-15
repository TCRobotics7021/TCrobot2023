// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AutoCubeAlign;
import frc.robot.commands.Drive.AutoMove;
import frc.robot.commands.Drive.DriveForward;
import frc.robot.commands.Drive.ResetFieldOrientation;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gripper.HomeGripper;
import frc.robot.commands.Gripper.autoGrip;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.UselessCommands.Blank_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test_Blue__2HighCube_MidCone extends SequentialCommandGroup {
  /** Creates a new Test_Blue_9Cone_HighCone_MidCone. */
  public Test_Blue__2HighCube_MidCone() {
   

    addCommands(
      new ResetFieldOrientation(),

    //Eject starting cube
      Commands.parallel(new setIntakeSpeed(Constants.intakeRevSpeed),
                        new CalibrateLiftAtStartOfMatch(Constants.liftStartingPOS)),
      new WaitCommand(.25), 

    //Move lift, gantry, and gripper to shift COG before moving
      new setLiftPosition(Constants.liftMaxLevelConePOS),
      new setGantryPosition(450),
      new Blank_Command().withTimeout(.05),
      new setGripperPosition(Constants.openGripperPOS),
      

    //Move back a little then turn 90 degrees
      Commands.parallel(
          new setGantryPosition(400), 
          new setLiftPosition(400),
          Commands.sequence(new AutoMove(.5, .6, 0, .1, .2, .2, 10, true, 0),
                            new setIntakeSpeed(0), 
                            new AutoMove(2.5, .7, 90, .1, .4, .4, 20, false, 0))),
    
    //Preparing to pick up second game piece
      Commands.parallel(new AutoMove(4, .3, 190, .1, .4, .1, 4, false, .1), 
                        Commands.sequence(new setGantryPosition(Constants.gantryPickPOS), 
                                          new setLiftPosition(Constants.liftBottomPOS))),
       new AutoCubeAlign(),
    
    //Picking up the second game piece
     Commands.parallel(new DriveForward(.25).withTimeout(1),
                       Commands.sequence(new setIntakeSpeed(Constants.intakeSpeed), 
                                          new autoGrip())),  
                        new Blank_Command().withTimeout(.05),
                        new setIntakeSpeed(Constants.intakeHoldingSpeed), 
  
    // Preparing to place second piece while moving back and turning around
    Commands.parallel(
                        Commands.sequence(new setLiftPosition(Constants.liftMidLevelCubePOS), 
                                          new setGantryPosition(Constants.gantryMidLevelPOS)),
                                          new AutoMove(1.5, .3, -20, .2, .4, .3, 6, false, 0)),
   
    // Placing the second game piece in the middle POS
    new AutoMove(.35, 0, 0, .2, .4, .05, 1, false, 0), 
    new setGripperPosition(Constants.openGripperPOS),
    new setIntakeSpeed(Constants.intakeRevSpeed),  
    new Blank_Command().withTimeout(.5), 

    // Drive back for the 3rd game piece
    Commands.parallel(new AutoMove(1.5, .7, 0, .1, .2, .2, 10, false, 0), 
                      new setGantryPosition(300), 
                      new setLiftPosition(750)),
    
    // Driving back while preparing to pick up next peice                  
    Commands.parallel(new AutoMove(3.7, .55, 135, .3, .5, .1, 3, false, 0), 
                      new HomeGripper(),
                      new setLiftPosition(Constants.liftRetrievePOS), 
                      new setGantryPosition(Constants.gantryPickPOS)),
        
    // Drive back with 3rd game piece
    new AutoMove(4.5, 0, 135, .2, .4, .1, 3, false, 0) 
    );
  }
}
