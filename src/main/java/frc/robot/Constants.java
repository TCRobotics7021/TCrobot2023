package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static int driveSettingTimeout = 100;
    public static int PIDindex = 0;

    public static double liftMotor_P = .1;
    public static double liftMotor_I = 0;
    public static double liftMotor_D = 0;

    //max and min lift output
    public static double liftOutputMax = 1;
    public static double liftOutputMin = -.8;
    public static double liftJogUp = .4;
    public static double liftJogDown = -.5;
    //max and min motor output
    

    //upper & lower limits
    public static double liftLowerLimit = 1;
    public static double liftUpperLimit = 1167;

    //lift conversion
    public static double liftConversion = 302.844;

      //upper and lower limit switch position / homing/calibrating values
      public static double liftLowerLimitSwitchPos = 0;
      public static double liftUpperLimitSwitchPos = 1170;

      public static final double liftPosTolerance = 5;

    // Back To home Variables
    public static double maxAutoRotate = 0;
    public static double minAutoRotate = 0;



    //Gripper subsystem
    public static double gripperMotor_P = .1;
    public static double gripperMotor_I = 0;
    public static double gripperMotor_D = 0;

    //max and min lift output
    public static double gripperOutputMax = .3;
    public static double gripperOutputMin = -.4;

    //upper & lower limits
    public static double gripperLowerLimit = 34;
    public static double gripperUpperLimit = 429;

    //lift conversion
    public static double gripperConversion = 112.7455;

    //upper and lower limit switch position / homing/calibrating values
    public static double gripperLowerLimitSwitchPos = 40;
    public static double gripperUpperLimitSwitchPos = 430;

    public static final double gripperPosTolerance = 5;

      //max and min arm output
      public static double ArmOutputMax = 1;
      public static double ArmOutputMin = -1;
  
      //upper & lower limits
      public static double ArmLowerLimit = 1;
      public static double ArmUpperLimit = 522;

      //arm conversion
      public static double ArmConversion = 307.122;
  
        //upper and lower limit switch position / homing/calibrating values
        public static double ArmLowerLimitSwitchPos = 0;
        public static double ArmUpperLimitSwitchPos = 523;
  
        public static final double ArmPosTolerance = 5;
        public static double ArmMotor_P = .1;
        public static double ArmMotor_I = 0;
        public static double ArmMotor_D = 0;
        
            //max and min arm output
      public static double GantryOutputMax = 1;
      public static double GantryOutputMin = -1;
  
      //upper & lower limits
      public static double GantryLowerLimit = 1;
      public static double GantryUpperLimit = 658;
  
      //gantry conversion
      public static double GantryConversion = 400.342;
  
        //upper and lower limit switch position / homing/calibrating values
        public static double GantryLowerLimitSwitchPos = 0;
        public static double GantryUpperLimitSwitchPos = 659;
  
        public static final double GantryPosTolerance = 5;
        public static double GantryMotor_P = .1;
        public static double GantryMotor_I = 0;
        public static double GantryMotor_D = 0;
        
    //GripperPOS
        public static double gripperCubeGrabPOS = 126;
        public static double gripperConeGrabPOS = 34;
        public static double openGripperPOS = 429;
        public static double gripperTimeout = .5;
        public static double gripperTimeoutCube = .1;
        public static double GripperStartingConeGripPos = 146;
        //gripper cali positions
        public static double GripperstartingConePos = 205;
        public static double GripperStartingcubePOS = 205;
        
    //ArmPOS
        public static double armRetractedPOS = 5;
        public static double armMidLevelPOS = 105;
        public static double armExtendedPOS = 500;
        public static double armPickPOS = 350;
        public static double armSideStation = 5;

    //GantryPOS
        public static double gantryLowerlevelPOS = 350;
        public static double gantryRetractedPOS = 1;
        public static double gantryPickPOS = 50;
        public static double gantryMidLevelPOS = 500;
        public static double gantryUpperLevelPOS = 650;
        public static double gantryClimbPOS = 1;
        public static double gantrySubPOS = 300;
        public static double gantrySideStation = 200;
    //LiftPOS
        public static double liftStartingPOS = 400;
        public static double liftBottomPOS = 0;
        public static double liftConeFlippy = 115;
        public static double liftRetrievePOS = 300;
        public static double liftLowerLevelPOS = 300;
        public static double liftMidLevelCubePOS = 700;
        public static double liftMidLevelConePOS = 900; 
        public static double liftMaxLevelCubePOS = 1075;
        public static double liftMaxLevelConePOS = 1167;
        public static double liftSubstationPOS = 940;
        public static double liftSubstationBumpingUpwardsPosition = 1000;
        public static double liftClimbPOS = 300;
        public static double liftSideStation = 700;
        public static double liftMaxLevelConeDip = 950;
        
        // Test Postions
    public static double gripperTestPOS = 100;
    public static double liftTestPOS = 200;
    public static double gantryTestPOS = 5;
    public static double armTestPOS = 5;
    
    //current limiting

    public static double LiftMaxCurrentAmps = 50;
    public static double LiftPeakCurrentAmps = 50;
    public static double LiftMaxCurrentTime = .5;

    public static double GantryMaxCurrentAmps = 20;
    public static double GantryPeakCurrentAmps = 25;
    public static double GantryMaxCurrentTime = .5;

    public static double GripperMaxCurrentAmps = 20;
    public static double GripperPeakCurrentAmps = 25;
    public static double GripperMaxCurrentTime = .5;

    public static double ArmMaxCurrentAmps = 20;
    public static double ArmPeakCurrentAmps = 25;
    public static double ArmMaxCurrentTime = .5;




    //Set speed for HomingLift
        public static double setSpeedForLiftHome = .5;

        public static double setSpeedForGripperHome = .25;

        public static double setSpeedforGantryHome = -.25;

        public static double setSpeedForArmHome = -.25;


        public static double gantryLimitLift = 350;
        public static double liftLimitGantry = 250;
   
    //DRIVE SECTION


    //minimum and max speed for setting auto motor speed
    public static double minSpeedPos = .2;
    public static double maxSpeedPos = .5;

    public static double minAutoRot = .2;
    public static double maxAutoRot = .6;

    public static double errorTolerance = .02;


    //P vaules
    public static final double autonomousMove_P = 1;
    public static final double autoRotate_P = .02;

    public static final double autoRotateTolerance = 2;

    //LimeLight
    public static double limeLightCubeAlignX = -10;
    public static double limeLightCubeALignY= 1.5;
    public static double limeLightCubeAlignP= 0.06;
    public static double limeLightCubeAlignP_Area= 1;
    public static double limeLightMinSpeed = .1;
    public static double limeLightMaxSpeed = .15;
    public static double limeLightXTolerance = 1;   
    public static double limeLightYTolerance = .1;   
    public static double limeLightATolerance = .1;


//charge station
  public static double climbStartedAngle = 7.5;
  public static double startedTiltDownAngle = 6.5;
  public static double balanceAngle = 4;

  public static double balanceTime = 3;
  public static double startClimbDelay = .5;
  public static double driveOverDelay = .4;

  public static double climbState0_StartingSpeed = .35;
  public static double climbState1_ClimbingSpeed = .15;
  public static double climbState2_Stopped = 0;
  public static double climbState3_REVspeed = -.11;
  public static double climbState4_FWDspeed = .11;

  public static double driveOverState0_StartingSpeed = .4;
  public static double driveOverState1_ClimbingSpeed = .3;
  public static double driveOverState2_DescendingSpeed = .3;


    public static final class Swerve {
        public static final int pigeonID = 9;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = .56; //TODO: This must be tuned to specific robot
        public static final double wheelBase = .68; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = Units.inchesToMeters(3.98) * Math.PI;//chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);//(.05/12);    //

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.5; //TODO: This must be tuned to specific robot (DONT TOUCH THE SPEED!!!!)
        public static final double driveSpeed = 4.5;
        public static final double fineSpeed = 2; 
        public static final double turboSpeed = 5.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0; //5.0 //TODO: This must be tuned to specific robot (DONT TOUCH THE SPEED!!!!)
        public static final double driveAngularVelocity = 5.0;
        public static final double fineAngularVelocity = 3;
        public static final double turboAngularVelocity = 5.0;
        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(347.344);//13.53
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(274.394);//85.86
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.869);//306.91
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(68.994);//286.78
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

   
   




    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        
    }
}
