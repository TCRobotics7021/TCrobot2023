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
import frc.robot.commands.Autonomous.Auto_Blue_1Cone_HighCube_LowCone;
import frc.robot.commands.Autonomous.Auto_Blue_1Cone_HighCube_LowCone_TEST;
import frc.robot.commands.Autonomous.Auto_Blue_2Cube_LowCone_LowCone;
import frc.robot.commands.Autonomous.Auto_Blue_5Cube_Overline_Climb;
import frc.robot.commands.Autonomous.Auto_Blue_8Cube_LowCone_LowCone;
import frc.robot.commands.Autonomous.Auto_Blue_9Cone_HighCube_Climb;
import frc.robot.commands.Autonomous.Auto_Blue_9Cone_HighCube_LowCube;
import frc.robot.commands.Autonomous.Auto_Red4Cone_HoldCone_Climb;
import frc.robot.commands.Autonomous.Auto_Red_1Cone_HighCube_LowCone;
import frc.robot.commands.Autonomous.Auto_Red_2Cube_LowCone_LowCone;
import frc.robot.commands.Autonomous.Auto_Red_5_Cube_Overline_Climb;
import frc.robot.commands.Autonomous.Auto_Red_8Cube_LowCone_LowCone;
import frc.robot.commands.Autonomous.Auto_Red_9Cone_HighCube_Climb;
import frc.robot.commands.Autonomous.Auto_Red_9Cone_HighCube_LowCone;
import frc.robot.commands.Autonomous.Auto_Red_9Cone_HighCube_LowCone_TEST;
import frc.robot.commands.Autonomous.DriveForward;
import frc.robot.commands.Autonomous.Place_Cone_Drive_Over;
import frc.robot.commands.Autonomous.Auto_Blue6Cone_HoldCone_Climb;
import frc.robot.commands.Autonomous.Auto_Blue_1Cone_HighCube_Climb;
import frc.robot.commands.Drive.AdvAutoMove;
import frc.robot.commands.Drive.AdvAutoMove2;
import frc.robot.commands.Drive.AdvAutoMove3;
import frc.robot.commands.Drive.GetOnChargeStationFromBack;
import frc.robot.commands.Drive.PrepareForClimb;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Drive.releaseAllBreaks;
import frc.robot.commands.Gantry.HomeGantry;
import frc.robot.commands.Gantry.setGantryPosition;
import frc.robot.commands.Gantry.setGantrySpeed;
import frc.robot.commands.Gripper.HomeGripper;
import frc.robot.commands.Gripper.releaseGripperBrake;
import frc.robot.commands.Gripper.setGripperPosition;
import frc.robot.commands.Gripper.setGripperSpeed;
import frc.robot.commands.Gripper.setIntakeSpeed;
import frc.robot.commands.Lift.HomeLift;
import frc.robot.commands.Lift.JogAndSetPOS;
import frc.robot.commands.Lift.releaseLiftBreak;
import frc.robot.commands.Lift.setLiftPosition;
import frc.robot.commands.PickPlace.CancelAll;
import frc.robot.commands.PickPlace.DropAndRetract;
import frc.robot.commands.PickPlace.HomeAll;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;
import frc.robot.commands.PickPlace.PlaceConeMidLevel;
import frc.robot.commands.PickPlace.PlaceObjectLowerLevel;
import frc.robot.commands.PickPlace.PlaceConeUpperLevel;
import frc.robot.commands.PickPlace.PrepareConeFlip;
import frc.robot.commands.PickPlace.PrepareForPickUp;
import frc.robot.commands.PickPlace.PrepareForSideStation;
import frc.robot.commands.PickPlace.PrepareForSubPickup;
import frc.robot.commands.PickPlace.ResetEndPlaceCommand;
import frc.robot.commands.PickPlace.RetrieveCone;
import frc.robot.commands.PickPlace.RetrieveCube;
import frc.robot.commands.PickPlace.RetrieveFromSub;
import frc.robot.commands.PickPlace.prepareCubePickup;
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

   // public final static CANdleSystem m_candleSubsystem = new CANdleSystem();
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
                () -> false,
                 () -> leftStick.getRawButton(2) //robot centric boolean
            )
        );


      
        m_Chooser.setDefaultOption("Place_Cone_Drive_Over_Line", new Place_Cone_Drive_Over());

      
        m_Chooser.addOption("Red_1Cone_HighCube_LowCone", new Auto_Red_1Cone_HighCube_LowCone());
        m_Chooser.addOption("Red_2Cube_LowCone_LowCone", new Auto_Red_2Cube_LowCone_LowCone());
        m_Chooser.addOption("Red 4Cone HoldCone Climb", new Auto_Red4Cone_HoldCone_Climb());
        m_Chooser.addOption("Red 5Cube Overline Climb", new Auto_Red_5_Cube_Overline_Climb());
        m_Chooser.addOption("Red_8Cube_LowCone_LowCone", new Auto_Red_8Cube_LowCone_LowCone());
      //  m_Chooser.addOption("Red_9Cone_HighCube_LowCone", new Auto_Red_9Cone_HighCube_LowCone());
        m_Chooser.addOption("TEST_Red_9Cone_HighCube_LowCone", new Auto_Red_9Cone_HighCube_LowCone_TEST());
        m_Chooser.addOption("Red_9Cone_HighCube_Climb", new Auto_Red_9Cone_HighCube_Climb());
        
    
        //m_Chooser.addOption("Blue_1Cone_HighCube_LowCone", new Auto_Blue_1Cone_HighCube_LowCone());
        m_Chooser.addOption("TEST_Blue_1Cone_HighCube_LowCone", new Auto_Blue_1Cone_HighCube_LowCone_TEST());
        m_Chooser.addOption("Blue_1Cone_HighCube_Climb", new Auto_Blue_1Cone_HighCube_Climb());
        m_Chooser.addOption("Blue_2Cube_LowCone_LowCone", new Auto_Blue_2Cube_LowCone_LowCone());
        m_Chooser.addOption("Blue 5Cube Overline Climb", new Auto_Blue_5Cube_Overline_Climb());
        m_Chooser.addOption("Blue 6Cone HoldCone Climb", new Auto_Blue6Cone_HoldCone_Climb());
        m_Chooser.addOption("Blue_8Cube_LowCone_LowCone", new Auto_Blue_8Cube_LowCone_LowCone());
        m_Chooser.addOption("Blue_9Cone_HighCube_LowCone", new Auto_Blue_9Cone_HighCube_LowCube());
        
        

   
       

         

        SmartDashboard.putData("Auto Chooser", m_Chooser);

        
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
  
        /* Driver Buttons */
        new JoystickButton(leftStick, 1).onTrue(new PrepareForPickUp().unless(() -> PlaceCommandStarted));
        new JoystickButton(leftStick, 2).onTrue(new prepareCubePickup().unless(() -> PlaceCommandStarted));
        new JoystickButton(leftStick, 3).onTrue(new PrepareConeFlip().unless(() -> PlaceCommandStarted));
        new JoystickButton(leftStick, 4).onTrue(new ConditionalCommand(new RetrieveFromSub(), new RetrieveCone(), s_Lift::liftGreaterThan200));
        new JoystickButton(leftStick, 10).whileTrue(new releaseGripperBrake());
       

        new JoystickButton(RightStick, 1).onTrue(new DropAndRetract());
        new JoystickButton(RightStick, 2).onTrue(new InstantCommand(() -> s_Swerve.Resetfieldorientation()));
        new JoystickButton(RightStick, 3).whileTrue(new releaseLiftBreak());
        new JoystickButton(RightStick, 4).whileTrue(new setIntakeSpeed(.5));
        new JoystickButton(RightStick, 5).whileTrue(new setIntakeSpeed(-.5));
        
        
     //  new JoystickButton(RightStick, 10).onTrue(new InstantCommand(() -> s_Candle.setMode(0)));
      //  new JoystickButton(OpPanel, 7).onTrue(new InstantCommand(() -> s_Candle.setMode(3)));
       // new JoystickButton(RightStick, 14).onTrue(new InstantCommand(() -> s_Candle.setMode(4)));
        //new JoystickButton(RightStick, 15).onTrue(new InstantCommand(() -> s_Candle.setMode(5)));
       // new JoystickButton(OpPanel,8).onTrue(new InstantCommand(() -> s_Candle.setMode(6)));

        new POVButton(RightStick, 0).whileTrue(new JogAndSetPOS(Constants.liftJogUp));
        new POVButton(RightStick, 45).whileTrue(new JogAndSetPOS(Constants.liftJogUp));
        new POVButton(RightStick, 315).whileTrue(new JogAndSetPOS(Constants.liftJogUp));
       
        new POVButton(RightStick, 225 ).whileTrue(new JogAndSetPOS(Constants.liftJogDown));
        new POVButton(RightStick, 135 ).whileTrue(new JogAndSetPOS(Constants.liftJogDown));
        new POVButton(RightStick, 180).whileTrue(new JogAndSetPOS(Constants.liftJogDown));


        
        //Operator Panel
        //new JoystickButton(OpPanel, 9).onTrue(new HomeAll());
      // new JoystickButton(OpPanel, 12).onTrue(new setLiftPosition(300));
       //new JoystickButton(OpPanel, 9).onTrue(new setLiftPosition(1000));
        new JoystickButton(OpPanel, 1).onTrue(new HomeAll());
        new JoystickButton(OpPanel, 2).onTrue(new GetOnChargeStationFromBack());
        new JoystickButton(OpPanel, 3).onTrue(new CancelAll());

        new JoystickButton(OpPanel, 5).onTrue(new PrepareForSubPickup());
        new JoystickButton(OpPanel, 6).onTrue(new PrepareForSideStation());

        
       // new JoystickButton(OpPanel, 9).onTrue(new DriveForward());

        new JoystickButton(OpPanel, 8).onTrue(new AdvAutoMove(2, 0, 180, .1, .5, .1, 5, true).withTimeout(5));
        new JoystickButton(OpPanel, 9).whileTrue(new setIntakeSpeed(.5));
        new JoystickButton(OpPanel, 12).whileTrue(new setIntakeSpeed(0));
       // new JoystickButton(OpPanel, 8).onTrue(new Auto_Blue9Cone8Cube());

       // new JoystickButton(OpPanel, 9).onTrue(new GetOnChargeStationFromBack());
      //  new JoystickButton(OpPanel, 9).onTrue(new Cube_Limelight_Test());

        //new JoystickButton(OpPanel, 12).whileTrue(new releaseAllBreaks());

        new JoystickButton(OpPanel, 14).onTrue(new PlaceObjectLowerLevel().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 15).onTrue(new PlaceConeMidLevel().unless(() -> PlaceCommandStarted));
        new JoystickButton(OpPanel, 16).onTrue(new PlaceConeUpperLevel().unless(() -> PlaceCommandStarted));
      //  new JoystickButton(OpPanel, 8 ).onTrue(new Auto_Blue_5Cube_Overline_Climb());
        
        

      //  new JoystickButton(OpPanel,7).whileTrue(new CameraAlignForCubePlace());
        
        

       //GetOnChargeStation
       // new JoystickButton(OpPanel, 5).onTrue(new ClimbOnly());
       // new JoystickButton(OpPanel, 6).onTrue(new PlaceConePOS1AndClimb());


        
        
       // new JoystickButton(OpPanel, 2).onTrue(new PrepareForClimb());
        
      
        //new JoystickButton(OpPanel, 5).onTrue(new MoveToPosReletiveToTarget(0.8, -.56, 0));
       
    
    
    
    }
        

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return (Command) m_Chooser.getSelected();
    }
}
