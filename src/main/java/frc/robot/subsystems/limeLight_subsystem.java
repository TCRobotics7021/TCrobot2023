// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limeLight_subsystem extends SubsystemBase {
//define variable and objects

  //NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight-mitch"); //change, maybe blinky
int delay = 0;

  /** Creates a new limeLight_subsystem. */
  public limeLight_subsystem() {

  

  }

//Functions:
public double tagRelativeXPos(){
  //return -limeLight.getEntry("botpose_targetspace").getDoubleArray(new double[3])[2];
  return 0;
}
public double tagRelativeYPos(){
  //return limeLight.getEntry("botpose_targetspace").getDoubleArray(new double[3])[0];
  return 0;
}
public double tagRelativeRPos(){
 //return limeLight.getEntry("botpose_targetspace").getDoubleArray(new double[3])[4];
 return 0;
}

//Reflective target position
public double targetX(){
 // return limeLight.getEntry("tx").getDouble(0);
 return 0;
}
public double targetY(){
//  return limeLight.getEntry("ty").getDouble(0);
return 0;
}
public double targetA(){
 // return limeLight.getEntry("ta").getDouble(0);
 return 0;
}


public void setPipeline(int pipelineNumber){
 // limeLight.getEntry("pipeline").setNumber(pipelineNumber); //Lets us change the vision pipeline
}

public void setLEDMode(int LEDNumber){
  //limeLight.getEntry("ledMode").setNumber(LEDNumber); 
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//if(delay > 50){
//SmartDashboard.putNumber("TagRelativeX", tagRelativeXPos());
//SmartDashboard.putNumber("TagRelativeY", tagRelativeYPos());
//SmartDashboard.putNumber("TagRelativeR", tagRelativeRPos());

//delay = 0;
  //}
 // delay++;



















 


  }
}


