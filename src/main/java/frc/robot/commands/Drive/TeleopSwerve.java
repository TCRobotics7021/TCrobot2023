package frc.robot.commands.Drive;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turbobutton;
    
    private double tempMaxSpeed = 0;
    private double tempMaxRotate = 0;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier turbobutton) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.turbobutton = turbobutton;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        SmartDashboard.putNumber("GYRO YAW", s_Swerve.getYaw().getDegrees());


    if (turbobutton.getAsBoolean()){
        if (RobotContainer.s_Arm.currentPosition() > 50) {

            tempMaxSpeed = Constants.Swerve.driveSpeed; 
            tempMaxRotate = Constants.Swerve.driveSpeed; 
    
        } else {
            tempMaxSpeed = Constants.Swerve.turboSpeed;
            tempMaxRotate = Constants.Swerve.turboSpeed;
        }
    }else{
        if (RobotContainer.s_Arm.currentPosition() > 50) {

            tempMaxSpeed = Constants.Swerve.fineSpeed; 
            tempMaxRotate = Constants.Swerve.fineAngularVelocity; 
    
        } else {
            tempMaxSpeed = Constants.Swerve.driveSpeed;
            tempMaxRotate = Constants.Swerve.driveSpeed;
        }
    }





        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(tempMaxSpeed), 
            rotationVal * tempMaxRotate,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}