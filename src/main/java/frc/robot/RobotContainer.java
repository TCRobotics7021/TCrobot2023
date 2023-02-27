package frc.robot;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.Drake;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Arm.HomeArm;
import frc.robot.commands.Arm.setArmPosition;
import frc.robot.commands.Arm.setArmSpeed;
import frc.robot.commands.Autonomous.AutoPlaceConeUpper;
import frc.robot.commands.Autonomous.PlaceConePOS1AndClimb;
import frc.robot.commands.Autonomous.PlaceConePosition1AndDriveOverLine;
import frc.robot.commands.Drive.ClimbOnly;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Gantry.HomeGantry;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gantry.setGantrySpeed;
import frc.robot.commands.Gripper.HomeGripper;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setGripperSpeed;
import frc.robot.commands.Lift.HomeLift;
import frc.robot.commands.Lift.JogAndSetPOS;
import frc.robot.commands.Lift.releaseLiftBreak;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.CancelAll;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.HomeAll;
import frc.robot.commands.PickPlace.PlaceConePOS1;
import frc.robot.commands.PickPlace.PlaceConePOS4;
import frc.robot.commands.PickPlace.PlaceObjectPOS7;
import frc.robot.commands.PickPlace.PlaceConePOS1;
import frc.robot.commands.PickPlace.PrepareConeFlip;
import frc.robot.commands.PickPlace.PrepareForPickUp;
import frc.robot.commands.PickPlace.PrepareForSubPickup;
import frc.robot.commands.PickPlace.ResetEndPlaceCommand;
import frc.robot.commands.PickPlace.RetrieveCone;
import frc.robot.commands.PickPlace.RetrieveCube;
import frc.robot.commands.PickPlace.RetrieveFromSub;
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
    SendableChooser m_Chooser = new SendableChooser<Command>();

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
        m_Chooser.setDefaultOption("Reset End Place Command", new ResetEndPlaceCommand());
        m_Chooser.addOption("Home All", new HomeAll());
        m_Chooser.addOption("Place Cone Upper", new AutoPlaceConeUpper());
        m_Chooser.addOption("Place Cone and Climb", new PlaceConePOS1AndClimb());
        m_Chooser.addOption("Place Cone POS 1 and Drive", new PlaceConePosition1AndDriveOverLine());

        SmartDashboard.putData("Auto CHooser", m_Chooser);
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(RightStick, 2).onTrue(new InstantCommand(() -> s_Swerve.Resetfieldorientation()));
        new JoystickButton(leftStick, 1).onTrue(new PrepareForPickUp().unless(() -> PlaceCommandStarted));
        new JoystickButton(RightStick, 3).whileTrue(new releaseLiftBreak());
        new JoystickButton(leftStick, 3).onTrue(new PrepareConeFlip().unless(() -> PlaceCommandStarted));
        new JoystickButton(RightStick, 1).onTrue(new DropAndRetract());
        new POVButton(RightStick, 0).whileTrue(new JogAndSetPOS(Constants.liftJogUp));
        new POVButton(RightStick, 180).whileTrue(new JogAndSetPOS(Constants.liftJogDown));
        //PlaceObjects
        new JoystickButton(OpPanel, 16).onTrue(new PlaceConePOS1().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 15).onTrue(new PlaceConePOS4().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 14).onTrue(new PlaceObjectPOS7().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 9).onTrue(new ClimbOnly());
        //PickupObjects
        new JoystickButton(leftStick, 4).onTrue(new ConditionalCommand(new RetrieveFromSub(), new RetrieveCone(), s_Lift::liftGreaterThan200));
        // new JoystickButton(leftStick, 4).onTrue(new ConditionalCommand(new RetrieveFromSub(), new RetrieveWithTipUp(), s_Lift::liftGreaterThan200));
       // new JoystickButton(RightStick, 4).onTrue(new RetrieveCube());

       //GetOnChargeStation
       // new JoystickButton(OpPanel, 5).onTrue(new ClimbOnly());
       // new JoystickButton(OpPanel, 6).onTrue(new PlaceConePOS1AndClimb());


        new JoystickButton(OpPanel, 1).onTrue(new HomeAll());
        new JoystickButton(OpPanel, 3).onTrue(new CancelAll());
        new JoystickButton(OpPanel, 2).onTrue(new PrepareForClimb());
        new JoystickButton(OpPanel, 5).onTrue(new PrepareForSubPickup());
      
        //new JoystickButton(OpPanel, 5).onTrue(new MoveToPosReletiveToTarget(0.8, -.56, 0));
    
    
    
    
    
    }
        

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return (Command) m_Chooser.getSelected();
    }
}
