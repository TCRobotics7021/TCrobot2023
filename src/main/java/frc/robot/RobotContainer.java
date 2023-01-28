package frc.robot;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
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
    private final Joystick OpPanel = new Joystick (2);

    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public final static limeLight_subsystem s_Limelight = new limeLight_subsystem();


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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(RightStick, 1).onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        new JoystickButton(OpPanel, 1).onTrue(new Calibrate().withTimeout(2));
        new JoystickButton(OpPanel, 2).onTrue(new AutonomousMove(0,2));
        new JoystickButton(OpPanel, 3).onTrue(new AutonomousMove(2,0));
        new JoystickButton(OpPanel, 4).onTrue(new AutonomousMove(2, 2));
        new JoystickButton(RightStick, 5).whileTrue(new ExactDrive(0, 1)); //Left
        new JoystickButton(RightStick, 6).whileTrue(new ExactDrive(1, 0)); //Forwards
        new JoystickButton(RightStick, 7).whileTrue(new ExactDrive(0, -1)); //Right
        new JoystickButton(RightStick, 8).whileTrue(new ExactDrive(-1, 0)); //Backwards
    }

    
    

    public Command ExactDrive(int i, int j) {
        return null;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
