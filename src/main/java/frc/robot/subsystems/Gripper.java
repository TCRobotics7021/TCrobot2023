// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  TalonFX m_Gripper = new TalonFX(12, "canivore1");


  private DigitalInput upperLimit = new DigitalInput(6);
  private DigitalInput lowerLimit = new DigitalInput(7);
  // private DigitalInput digitalLimit2 = new DigitalInput(2);
  // private DigitalInput digitalLimit3 = new DigitalInput(3);
  // private DigitalInput digitalLimit4 = new DigitalInput(4);
  // private DigitalInput digitalLimit5 = new DigitalInput(5);
  // private DigitalInput digitalLimit6 = new DigitalInput(6);
  // private DigitalInput digitalLimit7 = new DigitalInput(7);

  public double currentPosition () {
    return (m_Gripper.getSelectedSensorPosition(Constants.PIDindex)/Constants.gripperConversion);
  }


//variables 
private double tempP = 0;
private double tempI = 0;
private double tempD = 0;

private double tempPeakFWD = 0;
private double tempPeakREV = 0;





  public Gripper() {
    //settings for motors/variables
    m_Gripper.configFactoryDefault();
    m_Gripper.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDindex, Constants.driveSettingTimeout);
    m_Gripper.config_kP(Constants.PIDindex, Constants.gripperMotor_P, Constants.driveSettingTimeout);
    m_Gripper.config_kI(Constants.PIDindex, Constants.gripperMotor_I, Constants.driveSettingTimeout);
    m_Gripper.config_kD(Constants.PIDindex, Constants.gripperMotor_D, Constants.driveSettingTimeout);
    m_Gripper.configPeakOutputForward(Constants.gripperOutputMax, Constants.driveSettingTimeout);
    m_Gripper.configPeakOutputReverse(Constants.gripperOutputMin, Constants.driveSettingTimeout);
    m_Gripper.configAllowableClosedloopError(Constants.PIDindex, Constants.gripperPosTolerance, Constants.driveSettingTimeout);
    //in case the robot is doing the opposite of what we need (up or down)
    m_Gripper.setInverted(true); //set to true to flip positive direction
    //encoder reads positive
    m_Gripper.setSensorPhase(true);
    //adds physical limits
    m_Gripper.configForwardSoftLimitThreshold(Constants.gripperUpperLimit * Constants.gripperConversion, Constants.driveSettingTimeout );
    m_Gripper.configReverseSoftLimitThreshold(Constants.gripperLowerLimit * Constants.gripperConversion, Constants.driveSettingTimeout );
    //enables those limits
    m_Gripper.configForwardSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Gripper.configReverseSoftLimitEnable(true, Constants.driveSettingTimeout);
   
    tempP = Constants.gripperMotor_P;
    tempI = Constants.gripperMotor_I;
    tempD = Constants.gripperMotor_D;

    tempPeakFWD = Constants.gripperOutputMax;
    tempPeakREV = Constants.gripperOutputMin;

    // SmartDashboard.putNumber("P value", tempP);
    // SmartDashboard.putNumber("I value", tempI);
    // SmartDashboard.putNumber("D value", tempD);
    // SmartDashboard.putNumber("FWD Peak OutPut", tempPeakFWD);
    // SmartDashboard.putNumber("REV Peak OutPut", tempPeakREV); 
    // SmartDashboard.putBoolean("PID Tuning", false);
  } 


//Functions 
  public void setSpeed(double goSpeed ){
  m_Gripper.set(ControlMode.PercentOutput, goSpeed);
}

public void setPosition(double position){
  m_Gripper.set(ControlMode.Position, position*Constants.gripperConversion);

}
public boolean atTopLimit () {
return (!upperLimit.get());
}
  public boolean atBottomLimit () {
    return (!lowerLimit.get());
  }
public void calibrateEncoder (double calibratePosition) {
  m_Gripper.setSelectedSensorPosition(calibratePosition * Constants.gripperConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}

// != not equal
public void updatePID () {
if (SmartDashboard.getNumber("P value", tempP) != tempP) {
  tempP = SmartDashboard.getNumber("P value", tempP); 
  m_Gripper.config_kP(Constants.PIDindex, tempP, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("I value", tempI) != tempI) {
  tempP = SmartDashboard.getNumber("I value", tempI); 
  m_Gripper.config_kI(Constants.PIDindex, tempI, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("D value", tempD) != tempD) {
  tempP = SmartDashboard.getNumber("D value", tempD); 
  m_Gripper.config_kD(Constants.PIDindex, tempD, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD) != tempPeakFWD); {
  tempPeakFWD = SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD);
  m_Gripper.configPeakOutputForward(tempPeakFWD, Constants.driveSettingTimeout);
  }
  if (SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV) != tempPeakREV); {
    tempPeakREV = SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV);
    m_Gripper.configPeakOutputReverse(tempPeakREV, Constants.driveSettingTimeout);  
}
SmartDashboard.putNumber("Ticks", m_Gripper.getSelectedSensorPosition());
SmartDashboard.putNumber("Distance", m_Gripper.getSelectedSensorPosition()/Constants.gripperConversion);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(!lowerLimit.get()){
     m_Gripper.setSelectedSensorPosition(Constants.gripperLowerLimitSwitchPos * Constants.gripperConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(!upperLimit.get()){
     m_Gripper.setSelectedSensorPosition(Constants.gripperUpperLimitSwitchPos * Constants.gripperConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    
    //updatePID();
 
    SmartDashboard.putBoolean("gripperUpperLimit", upperLimit.get());
    SmartDashboard.putBoolean("gripperLowerLimit", lowerLimit.get());
   
    
  }
}
