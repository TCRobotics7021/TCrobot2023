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

public class Arm extends SubsystemBase {
    // Creates a new Arm
  WPI_TalonFX m_Arm = new WPI_TalonFX(14, "canivore1");


  private DigitalInput upperLimit = new DigitalInput(0);
  private DigitalInput lowerLimit = new DigitalInput(1);
  
  public double currentPosition () {
    return (m_Arm.getSelectedSensorPosition(Constants.PIDindex)/Constants.ArmConversion);
  }


//variables 
private double tempP = 0;
private double tempI = 0;
private double tempD = 0;

private double tempPeakFWD = 0;
private double tempPeakREV = 0;





  public Arm() {
    //settings for motors/variables
    m_Arm.configFactoryDefault();
    m_Arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PIDindex, Constants.driveSettingTimeout);
    m_Arm.config_kP(Constants.PIDindex, Constants.ArmMotor_P, Constants.driveSettingTimeout);
    m_Arm.config_kI(Constants.PIDindex, Constants.ArmMotor_I, Constants.driveSettingTimeout);
    m_Arm.config_kD(Constants.PIDindex, Constants.ArmMotor_D, Constants.driveSettingTimeout);
    m_Arm.configPeakOutputForward(Constants.ArmOutputMax, Constants.driveSettingTimeout);
    m_Arm.configPeakOutputReverse(Constants.ArmOutputMin, Constants.driveSettingTimeout);
    m_Arm.configAllowableClosedloopError(Constants.PIDindex, Constants.ArmPosTolerance, Constants.driveSettingTimeout);
    //in case the robot is doing the opposite of what we need (up or down)
    m_Arm.setInverted(true); //set to true to flip positive direction
    //encoder reads positive
    m_Arm.setSensorPhase(true);
    //adds physical limits
    m_Arm.configForwardSoftLimitThreshold(Constants.ArmUpperLimit * Constants.ArmConversion, Constants.driveSettingTimeout );
    m_Arm.configReverseSoftLimitThreshold(Constants.ArmLowerLimit * Constants.ArmConversion, Constants.driveSettingTimeout );
    //enables those limits
    m_Arm.configForwardSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Arm.configReverseSoftLimitEnable(true, Constants.driveSettingTimeout);
    m_Arm.setNeutralMode(NeutralMode.Brake);

     tempP = Constants.ArmMotor_P;
    tempI = Constants.ArmMotor_I;
    tempD = Constants.ArmMotor_D;

    tempPeakFWD = Constants.ArmOutputMax;
    tempPeakREV = Constants.ArmOutputMin;

    // SmartDashboard.putNumber("P value", tempP);
    // SmartDashboard.putNumber("I value", tempI);
    // SmartDashboard.putNumber("D value", tempD);
    // SmartDashboard.putNumber("FWD Peak OutPut", tempPeakFWD);
    // SmartDashboard.putNumber("REV Peak OutPut", tempPeakREV); 
    // SmartDashboard.putBoolean("PID Tuning", false);
   } 


//Functions 
  public void setSpeed(double goSpeed ){
    if(RobotContainer.s_Lift.currentPosition() > Constants.liftLimitGantry){
         m_Arm.set(ControlMode.PercentOutput, goSpeed);
    }
}

public void setPosition(double position){
  if(RobotContainer.s_Lift.currentPosition() > Constants.liftLimitGantry){
        m_Arm.set(ControlMode.Position, position*Constants.ArmConversion);
  }
}
public boolean atTopLimit () {
return (upperLimit.get());
}
  public boolean atBottomLimit () {
    return (lowerLimit.get());
  }
public void calibrateEncoder (double calibratePosition) {
  m_Arm.setSelectedSensorPosition(calibratePosition * Constants.ArmConversion, Constants.PIDindex, Constants.driveSettingTimeout);
}

public void setCoastMode() {
  m_Arm.setNeutralMode(NeutralMode.Coast);
}

public void setBrakeMode() {
  m_Arm.setNeutralMode(NeutralMode.Brake);
}

// != not equal
public void updatePID () {
if (SmartDashboard.getNumber("P value", tempP) != tempP) {
  tempP = SmartDashboard.getNumber("P value", tempP); 
  m_Arm.config_kP(Constants.PIDindex, tempP, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("I value", tempI) != tempI) {
  tempP = SmartDashboard.getNumber("I value", tempI); 
  m_Arm.config_kI(Constants.PIDindex, tempI, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("D value", tempD) != tempD) {
  tempP = SmartDashboard.getNumber("D value", tempD); 
  m_Arm.config_kD(Constants.PIDindex, tempD, Constants.driveSettingTimeout);
}
if (SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD) != tempPeakFWD); {
  tempPeakFWD = SmartDashboard.getNumber("FWD Peak OutPut", tempPeakFWD);
  m_Arm.configPeakOutputForward(tempPeakFWD, Constants.driveSettingTimeout);
  }
  if (SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV) != tempPeakREV); {
    tempPeakREV = SmartDashboard.getNumber("REV Peak OutPut", tempPeakREV);
    m_Arm.configPeakOutputReverse(tempPeakREV, Constants.driveSettingTimeout);  
}
SmartDashboard.putNumber("Ticks", m_Arm.getSelectedSensorPosition());
SmartDashboard.putNumber("Distance", m_Arm.getSelectedSensorPosition()/Constants.ArmConversion);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(lowerLimit.get()){
      m_Arm.setSelectedSensorPosition(Constants.ArmLowerLimitSwitchPos * Constants.ArmConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }

    if(upperLimit.get()){
      m_Arm.setSelectedSensorPosition(Constants.ArmUpperLimitSwitchPos * Constants.ArmConversion, Constants.PIDindex, Constants.driveSettingTimeout);
    }
    
    //Stops the arm from moving when the lift is below limit
    if(RobotContainer.s_Lift.currentPosition() <= Constants.liftLimitGantry){
      m_Arm.set(ControlMode.PercentOutput, 0);
    }
 
    //updatePID();
    
    SmartDashboard.putBoolean("ArmUpperLimit", upperLimit.get());
    SmartDashboard.putBoolean("ArmLowerLimit", lowerLimit.get());
    SmartDashboard.putNumber("Arm Position", m_Arm.getSelectedSensorPosition()/Constants.ArmConversion);
 
 
  }
}
