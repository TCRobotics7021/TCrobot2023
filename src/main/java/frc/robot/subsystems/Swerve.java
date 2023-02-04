package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDriveOdometry tempOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        SmartDashboard.putNumber("path X1", 0);
        SmartDashboard.putNumber("path Y1", 0);
        SmartDashboard.putNumber("path R1", 0);
        SmartDashboard.putNumber("path X2", .5);
        SmartDashboard.putNumber("path Y2", 0);
        SmartDashboard.putNumber("path X3", 1);
        SmartDashboard.putNumber("path Y3", 0);
        SmartDashboard.putNumber("path X4", 1.5);
        SmartDashboard.putNumber("path Y4", 0);
        SmartDashboard.putNumber("path R4", 0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        Timer.delay(1.0);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        tempOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation2d.getX(), 
                                    translation2d.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation2d.getX(), 
                                    translation2d.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    public Pose2d gettempPose() {
        return tempOdometry.getPoseMeters();
    }


    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }
    public void resettempOdometry(Pose2d pose) {
        tempOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }


    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void Resetfieldorientation(){
        gyro.setYaw(0);
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));


    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        
        swerveOdometry.update(getYaw(), getModulePositions());  
        tempOdometry.update(getYaw(), getModulePositions());

        SmartDashboard.putNumber("Odometry X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry R", swerveOdometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Temp Odometry X", tempOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Temp Odometry Y", tempOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Temp Odometry R", tempOdometry.getPoseMeters().getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
        }
    }

    public void drive(Rotation2d times, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    }

    public Object Resetfieldorientation() {
        return null;
    }
}