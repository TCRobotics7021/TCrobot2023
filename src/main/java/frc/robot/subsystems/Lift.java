// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Lift.setLiftPosition;

public class Lift extends SubsystemBase {
    // Creates a new Lift
  WPI_TalonFX m_Lift = new WPI_TalonFX(11, "canivore1");


  private DigitalInput upperLimit = new DigitalInput(4);
  private DigitalInput lowerLimit = new DigitalInput(3);
  public double currentPosition () {
    return (m_Lift.getSelectedSensorPosition(Constants.PIDindex)/Constants.liftConversion);
  }


//variables 
private double tempP = 0;
private double tempI = 0;
private double tempD = 0;

private double tempPeakFWD = 0;
private double tempPeakREV = 0;

private double tempLowerLimit = 0;



  public Lift() {
    //settings for motors/variables
    m_Lift.configFactoryDefault();
    m_Lift.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDindex, Constants.driveSettingTimeout);
    m_Lift.config_kP(Constants.PIDindex, Constants.liftMotor_P, Constants.driveSettingTimeout);
    m_Lift.config_kI(Constants.PIDindex, Constants.liftMotor_I, Constants.driveSettingTimeout);
    m_Lift.config_kD(Constants.PIDindex, Constants.liftMotor_D, Constants.driveSettingTimeout);
    m_Lift.configPeakOutputForward(Constants.liftOutputMax, Constants.driveSettingTimeout);
    m_Lift.configPeakOutputReverse(Constants.liftOutputMin, Constants.driveSettingTimeout);
    m_Lift.configAllowableClosedloopError(Constants.PIDindex, Constants.liftPosTolerance, Constants.driveSettingTimeout);
    //in case the robot is doing the opposite of what we need (up or down)
    m_Lift.setInverted(false); //set to true to flip positive direction
    //encoder reads positive
    m_Lift.setSensorPhase(true);
    //adds physical limits
    m_Lift.configForwardSoftLimitThreshold(Constants.liftUpperLimit * Constants.liftConversion, Constants.driveSettingTimeout );
    m_Lift.configReverseSoftLimitThreshold(Constants.liftLowerLimit * Constants.liftConversion, Constants.driveSettingTimeout );
    //enables those limits
    m_Lift.configForwardSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Lift.configReverseSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Lift.setNeutralMode(NeutralMode.Brake);
    calibrateEncoder(Constants.liftStartingPOS);

    tempP = Constants.liftMotor_P;
    tempI = Constants.liftMotor_I;
    tempD = Constants.liftMotor_D;

    tempPeakFWD = Constants.liftOutputMax;
    tempPeakREV = Constants.liftOutputMin;

    // SmartDashboard.putNumber("P value", tempP);
    // SmartDashboard.putNumber("I value", tempI);
    // SmartDashboard.putNumber("D value", tempD);
    // SmartDashboard.putNumber("FWD Peak OutPut", tempPeakFWD);
    // SmartDashboard.putNumber("REV Peak OutPut", tempPeakREV); 
    // SmartDashboard.putBoolean("PID Tuning", false);
   } 


//Functions 
  public void setSpeed(double goSpeed ){
  m_Lift.set(ControlMode.PercentOutput, goSpeed);
}
  public boolean liftGreaterThan200() {
    return currentPosition() > 200;
  }
public void setPosition(double position){
  m_Lift.set(ControlMode.Position, position*Constants.liftConversion);
}
public boolean atTopLimit () {
return (!upperLimit.get());
}
  public boolean atBottomLimit () {
    return (!lowerLimit.get());
  }
public void calibrateEncoder (double calibratePosition) {
  m_Lift.setSelectedSensorPosition(calibratePosition * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}

public void setCoastMode() {
  m_Lift.setNeutralMode(NeutralMode.Coast);
}

public void setBrakeMode() {
  m_Lift.setNeutralMode(NeutralMode.Brake);
}

// != not equal
public void updatePID () {
if (SmartDashboard.getNumber("P value", tempP) != tempP) {
  tempP = SmartDashboard.getNumber("P value", tempP); 
  m_Lift.config_kP(Constants.PIDindex, tempP, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("I value", tempI) != tempI) {
  tempP = SmartDashboard.getNumber("I value", tempI); 
  m_Lift.config_kI(Constants.PIDindex, tempI, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("D value", tempD) != tempD) {
  tempP = SmartDashboard.getNumber("D value", tempD); 
  m_Lift.config_kD(Constants.PIDindex, tempD, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD) != tempPeakFWD); {
  tempPeakFWD = SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD);
  m_Lift.configPeakOutputForward(tempPeakFWD, Constants.driveSettingTimeout);
  }
  if (SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV) != tempPeakREV); {
    tempPeakREV = SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV);
    m_Lift.configPeakOutputReverse(tempPeakREV, Constants.driveSettingTimeout);  
}
SmartDashboard.putNumber("Ticks", m_Lift.getSelectedSensorPosition());
SmartDashboard.putNumber("Distance", m_Lift.getSelectedSensorPosition()/Constants.liftConversion);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(!lowerLimit.get()){
      m_Lift.setSelectedSensorPosition(Constants.liftLowerLimitSwitchPos * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(!upperLimit.get()){
      m_Lift.setSelectedSensorPosition(Constants.liftUpperLimitSwitchPos * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(RobotContainer.s_Gantry.currentPosition() + RobotContainer.s_Arm.currentPosition() <= Constants.gantryLimitLift && currentPosition() < Constants.liftLimitGantry && m_Lift.getMotorOutputPercent() < 0){
      setPosition(Constants.liftLimitGantry + 5);
    }
    
    
    //updatePID();
    
    SmartDashboard.putBoolean("liftUpperLimit", upperLimit.get());
    SmartDashboard.putBoolean("liftLowerLimit", lowerLimit.get());
   // SmartDashboard.putNumber("Output", m_Lift.getMotorOutputPercent());
 
   SmartDashboard.putNumber("Lift Position", m_Lift.getSelectedSensorPosition()/Constants.liftConversion);
 
  }
}
