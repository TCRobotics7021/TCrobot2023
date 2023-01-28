// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    // Creates a new Lift
  WPI_TalonFX m_Lift = new WPI_TalonFX(11);

  DigitalInput UpperLimit = new DigitalInput(1);
  DigitalInput LowerLimit = new DigitalInput(0);

  private WPI_TalonFX m_lift = new WPI_TalonFX(11);
  private DigitalInput upperLimit = new DigitalInput(1);
  private DigitalInput lowerLimit = new DigitalInput(0);
  public double currentPosition () {
    return (m_Lift.getSelectedSensorPosition(Constants.PIDindex)/Constants.liftConversion);
  }


//variables 
private double tempP = 0;
private double tempI = 0;
private double tempD = 0;

private double tempPeakFWD = 0;
private double tempPeakREV = 0;





  public Lift() {
    //settings for motors/variables
    m_lift.configFactoryDefault();
    m_lift.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDindex, Constants.driveSettingTimeout);
    m_lift.config_kP(Constants.PIDindex, Constants.liftMotor_P, Constants.driveSettingTimeout);
    m_lift.config_kI(Constants.PIDindex, Constants.liftMotor_I, Constants.driveSettingTimeout);
    m_lift.config_kD(Constants.PIDindex, Constants.liftMotor_D, Constants.driveSettingTimeout);
    m_lift.configPeakOutputForward(Constants.liftOutputMax * Constants.liftConversion, Constants.driveSettingTimeout);
    m_lift.configPeakOutputReverse(Constants.liftOutputMin * Constants.liftConversion, Constants.driveSettingTimeout);
    m_lift.configAllowableClosedloopError(Constants.PIDindex, Constants.liftPosTolerance, Constants.driveSettingTimeout);
    //in case the robot is doing the opposite of what we need (up or down)
    m_lift.setInverted(false); //set to true to flip positive direction
    //encoder reads positive
    m_lift.setSensorPhase(true);
    //adds physical limits
    m_lift.configForwardSoftLimitThreshold(Constants.liftUpperLimit * Constants.liftConversion, Constants.driveSettingTimeout );
    m_lift.configReverseSoftLimitThreshold(Constants.liftLowerLimit * Constants.liftConversion, Constants.driveSettingTimeout );
    //enables those limits
    m_lift.configForwardSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_lift.configReverseSoftLimitEnable(true, Constants.driveSettingTimeout);
   
    tempP = Constants.liftMotor_P;
    tempI = Constants.liftMotor_I;
    tempD = Constants.liftMotor_P;

    tempPeakFWD = Constants.liftOutputMax;
    tempPeakREV = Constants.liftOutputMin;

    SmartDashboard.putNumber("P value", tempP);
    SmartDashboard.putNumber("I value", tempI);
    SmartDashboard.putNumber("D value", tempD);
    SmartDashboard.putNumber("FWD Peak OutPut", tempPeakFWD);
    SmartDashboard.putNumber("REV Prak OutPut", tempPeakREV); 
    SmartDashboard.putBoolean("PID Tuning", false);
  } 


//Functions 
  public void setSpeed(double goSpeed ){
  m_lift.set(ControlMode.PercentOutput, goSpeed);
}

public void setPosition(double calibratePosition){
  m_lift.setSelectedSensorPosition(calibratePosition * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}
public boolean atTopLimit () {
return (!LowerLimit.get());
}
  public boolean atBottomLimit () {
    return (!LowerLimit.get());
  }
public void calibrateEncoder (double calibratePosition) {
  m_lift.setSelectedSensorPosition(calibratePosition * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}
// != not equal
public void updatePID () {
if (SmartDashboard.getNumber("P value", tempP) != tempP) {
  tempP = SmartDashboard.getNumber("P value", tempP); 
  m_lift.config_kP(Constants.PIDindex, tempP, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("I value", tempI) != tempI) {
  tempP = SmartDashboard.getNumber("I value", tempI); 
  m_lift.config_kI(Constants.PIDindex, tempI, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("D value", tempD) != tempD) {
  tempP = SmartDashboard.getNumber("D value", tempD); 
  m_lift.config_kD(Constants.PIDindex, tempD, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD) != tempPeakFWD); {
  tempPeakFWD = SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD);
  m_lift.configPeakOutputForward(tempPeakFWD, Constants.driveSettingTimeout);
  }
  if (SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV) != tempPeakREV); {
    tempPeakFWD = SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV);
    m_lift.configPeakOutputForward(tempPeakREV, Constants.driveSettingTimeout);  
}
SmartDashboard.putNumber("Ticks", m_lift.getSelectedSensorPosition());
SmartDashboard.putNumber("Distance", m_lift.getSelectedSensorPosition()/Constants.liftConversion);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(!lowerLimit.get()){
      m_lift.setSelectedSensorPosition(Constants.liftLowerLimitSwitchPos * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(!upperLimit.get()){
      m_lift.setSelectedSensorPosition(Constants.liftUpperLimitSwitchPos * Constants.liftConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if (SmartDashboard.getBoolean("PID Value", false)) {
      updatePID();
    }
 
 
 
 
  }
}
