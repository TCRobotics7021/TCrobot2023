// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Gantry extends SubsystemBase {
  /** Creates a new Gantry. */
  TalonFX m_Gantry = new TalonFX(13, "canivore1");



  private DigitalInput upperLimit = new DigitalInput(5);
  private DigitalInput lowerLimit = new DigitalInput(2);
  // private DigitalInput digitalLimit2 = new DigitalInput(2);
  // private DigitalInput digitalLimit3 = new DigitalInput(3);
  // private DigitalInput digitalLimit4 = new DigitalInput(4);
  // private DigitalInput digitalLimit5 = new DigitalInput(5);
  // private DigitalInput digitalLimit6 = new DigitalInput(6);
  // private DigitalInput digitalLimit7 = new DigitalInput(7);

  public double currentPosition () {
    return (m_Gantry.getSelectedSensorPosition(Constants.PIDindex)/Constants.GantryConversion);
  }


//variables 
private double tempP = 0;
private double tempI = 0;
private double tempD = 0;

private double tempPeakFWD = 0;
private double tempPeakREV = 0;
private double tempLowerLimit = 0;





  public Gantry() {
    //settings for motors/variables
    m_Gantry.configFactoryDefault();
    m_Gantry.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDindex, Constants.driveSettingTimeout);
    m_Gantry.config_kP(Constants.PIDindex, Constants.GantryMotor_P, Constants.driveSettingTimeout);
    m_Gantry.config_kI(Constants.PIDindex, Constants.GantryMotor_I, Constants.driveSettingTimeout);
    m_Gantry.config_kD(Constants.PIDindex, Constants.GantryMotor_D, Constants.driveSettingTimeout);
    m_Gantry.configPeakOutputForward(Constants.GantryOutputMax, Constants.driveSettingTimeout);
    m_Gantry.configPeakOutputReverse(Constants.GantryOutputMin, Constants.driveSettingTimeout);
    m_Gantry.configAllowableClosedloopError(Constants.PIDindex, Constants.GantryPosTolerance, Constants.driveSettingTimeout);
    //in case the robot is doing the opposite of what we need (up or down)
    m_Gantry.setInverted(true); //set to true to flip positive direction
    //encoder reads positive
    m_Gantry.setSensorPhase(true);
    //adds physical limits
    m_Gantry.configForwardSoftLimitThreshold(Constants.GantryUpperLimit * Constants.GantryConversion, Constants.driveSettingTimeout );
    m_Gantry.configReverseSoftLimitThreshold(Constants.GantryLowerLimit * Constants.GantryConversion, Constants.driveSettingTimeout );
    //enables those limits
    m_Gantry.configForwardSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Gantry.configReverseSoftLimitEnable(true, Constants.driveSettingTimeout);
   
    tempP = Constants.GantryMotor_P;
    tempI = Constants.GantryMotor_I;
    tempD = Constants.GantryMotor_D;

    tempPeakFWD = Constants.GantryOutputMax;
    tempPeakREV = Constants.GantryOutputMin;
    m_Gantry.setNeutralMode(NeutralMode.Brake);
   // m_Gantry.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.GantryMaxCurrentAmps, Constants.GantryPeakCurrentAmps, Constants.GantryMaxCurrentTime));
    
    tempLowerLimit = Constants.GantryLowerLimit;
    // //SmartDashboard.putNumber("P value", tempP);
    // SmartDashboard.putNumber("I value", tempI);
    // SmartDashboard.putNumber("D value", tempD);
    // SmartDashboard.putNumber("FWD Peak OutPut", tempPeakFWD);
    // SmartDashboard.putNumber("REV Peak OutPut", tempPeakREV); 
    // SmartDashboard.putBoolean("PID Tuning", false);
  } 


//Functions 
  public void setSpeed(double goSpeed ){
  if(RobotContainer.s_Lift.currentPosition() > Constants.liftLimitGantry){
    m_Gantry.set(ControlMode.PercentOutput, goSpeed);
  }
}

 

public void setPosition(double position){
  if(RobotContainer.s_Lift.currentPosition() > Constants.liftLimitGantry){
      m_Gantry.set(ControlMode.Position, position*Constants.GantryConversion);
  }
}
public boolean atTopLimit () {
return (!upperLimit.get());
}
  public boolean atBottomLimit () {
    return (!lowerLimit.get());
  }
public void calibrateEncoder (double calibratePosition) {
  m_Gantry.setSelectedSensorPosition(calibratePosition * Constants.GantryConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}

public void setCoastMode() {
  m_Gantry.setNeutralMode(NeutralMode.Coast);
}

public void setBrakeMode() {
  m_Gantry.setNeutralMode(NeutralMode.Brake);
}


// != not equal
public void updatePID () {
if (SmartDashboard.getNumber("P value", tempP) != tempP) {
  tempP = SmartDashboard.getNumber("P value", tempP); 
  m_Gantry.config_kP(Constants.PIDindex, tempP, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("I value", tempI) != tempI) {
  tempI = SmartDashboard.getNumber("I value", tempI); 
  m_Gantry.config_kI(Constants.PIDindex, tempI, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("D value", tempD) != tempD) {
  tempD = SmartDashboard.getNumber("D value", tempD); 
  m_Gantry.config_kD(Constants.PIDindex, tempD, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD) != tempPeakFWD); {
  tempPeakFWD = SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD);
  m_Gantry.configPeakOutputForward(tempPeakFWD, Constants.driveSettingTimeout);
  }
  if (SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV) != tempPeakREV); {
    tempPeakREV = SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV);
    m_Gantry.configPeakOutputReverse(tempPeakREV, Constants.driveSettingTimeout);  
}
//SmartDashboard.putNumber("Ticks", m_Gantry.getSelectedSensorPosition());
//SmartDashboard.putNumber("Distance", m_Gantry.getSelectedSensorPosition()/Constants.GantryConversion);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(!lowerLimit.get()){
     m_Gantry.setSelectedSensorPosition(Constants.GantryLowerLimitSwitchPos * Constants.GantryConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(!upperLimit.get()){
     m_Gantry.setSelectedSensorPosition(Constants.GantryUpperLimitSwitchPos * Constants.GantryConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(RobotContainer.s_Lift.currentPosition() <= Constants.liftLimitGantry){
      m_Gantry.set(ControlMode.PercentOutput, 0);
    }
    

   //updatePID();
    //SmartDashboard.putNumber("Gantry Stator Current", m_Gantry.getStatorCurrent());
    SmartDashboard.putBoolean("GantryUpperLimit", upperLimit.get());
    SmartDashboard.putBoolean("GantryLowerLimit", lowerLimit.get());
   SmartDashboard.putNumber("Gantry Position", m_Gantry.getSelectedSensorPosition()/Constants.GantryConversion);
    
  }
}

