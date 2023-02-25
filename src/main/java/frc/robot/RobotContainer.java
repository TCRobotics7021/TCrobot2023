package frc.robot;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Arm.setArmSpeed;
import frc.robot.commands.Autonomous.ClimbOnly;
import frc.robot.commands.Gantry.HomeGantry;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gantry.setGantrySpeed;
import frc.robot.commands.Gripper.HomeGripper;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setGripperSpeed;
import frc.robot.commands.Lift.HomeLift;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.CancelAll;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.HomeAll;
import frc.robot.commands.PickPlace.PlaceConePOS1;
import frc.robot.commands.PickPlace.PlaceConePOS4;
import frc.robot.commands.PickPlace.PlaceConePOS1;
import frc.robot.commands.PickPlace.PrepareConeFlip;
import frc.robot.commands.PickPlace.PrepareForPickUp;
import frc.robot.commands.PickPlace.RetrieveCone;
import frc.robot.commands.PickPlace.RetrieveCube;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick  RightStick = new Joystick(1);
    private final Joystick leftStick = new Joystick(0);
    private final Joystick OpPanel = new Joystick(2);

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public final static limeLight_subsystem s_Limelight = new limeLight_subsystem();
    public final static Lift s_Lift = new Lift();
    public final static Gripper s_Gripper = new Gripper();
    public final static Gantry s_Gantry = new Gantry();
    public final static Arm s_Arm = new Arm();
    public static boolean EndPlaceCommand = false;
    public static boolean PlaceCommandStarted = false;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> leftStick.getRawAxis(1), //translational x
                () -> leftStick.getRawAxis(0),  //translational y
                () -> RightStick.getRawAxis(0), //rotational
                () -> false  //robot centric boolean
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(RightStick, 2).onTrue(new InstantCommand(() -> s_Swerve.Resetfieldorientation()));
        new JoystickButton(leftStick, 1).onTrue(new PrepareForPickUp().unless(() -> PlaceCommandStarted));
        
        new JoystickButton(leftStick, 3).onTrue(new PrepareConeFlip().unless(() -> PlaceCommandStarted));
        new JoystickButton(RightStick, 1).onTrue(new DropAndRetract());
        //PlaceObjects
        new JoystickButton(OpPanel, 16).onTrue(new PlaceConePOS1().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 15).onTrue(new PlaceConePOS4().unless(() -> PlaceCommandStarted));
       
        //PickupObjects
        new JoystickButton(leftStick, 4).onTrue(new RetrieveCone().unless(() -> PlaceCommandStarted));
       // new JoystickButton(RightStick, 4).onTrue(new RetrieveCube());

       //GetOnChargeStation
        new JoystickButton(OpPanel, 5).onTrue(new ClimbOnly());


        new JoystickButton(OpPanel, 1).onTrue(new HomeAll());
        new JoystickButton(OpPanel, 3).onTrue(new CancelAll());
        new JoystickButton(OpPanel, 2).whileTrue(new releaseLiftBreak());
        
        //new JoystickButton(OpPanel, 5).onTrue(new MoveToPosReletiveToTarget(0.8, -.56, 0));
    
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    }
        

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
