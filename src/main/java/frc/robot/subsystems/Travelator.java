// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;


public class Travelator extends SubsystemBase {
  // ***** Create Motors ***** //
  public WPI_TalonFX Travelator = new WPI_TalonFX(opConstants.kTravelatorID);
  public DigitalInput LimitSwitch1 = new DigitalInput(opConstants.kBackRightLimitSwitchID);
  public DigitalInput LimitSwitch2 = new DigitalInput(opConstants.kBackLeftLimitSwitchID);
  public DigitalInput LimitSwitch3 = new DigitalInput(opConstants.kFrontRightLimitSwitchID);
  public DigitalInput LimitSwitch4 = new DigitalInput(opConstants.kFrontLeftLimitSwitchID);

  public Travelator() {
    addChild("Motor",Travelator);
    
    addChild("Switch 1", LimitSwitch1);
    addChild("Switch 2", LimitSwitch2);
    addChild("Switch 3", LimitSwitch3);
    addChild("Switch 4", LimitSwitch4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putData("Switch1", LimitSwitch1);
    // SmartDashboard.putData("Switch2", LimitSwitch2);
    SmartDashboard.putNumber("Travelator Pose", getTravelatorPos());
    SmartDashboard.updateValues();

    if(!LimitSwitch1.get() || !LimitSwitch2.get()){
      Travelator.setSelectedSensorPosition(opConstants.kTravelatorMinTicks);
      Travelator.set(ControlMode.PercentOutput, 0);
    }

    if(!LimitSwitch3.get() || !LimitSwitch4.get()){
      // Travelator.setSelectedSensorPosition(opConstants.kTravelatorMaxTicks);
      Travelator.set(ControlMode.PercentOutput, 0);
    }
  }

  public void Moveforward() {
    if (!LimitSwitch1.get() || !LimitSwitch2.get()) {
      Travelator.set(ControlMode.PercentOutput, 0);
     }
     else {
      Travelator.set(ControlMode.PercentOutput, opConstants.kTravelatorSpeed);
     }
  }

  public void MoveBackward() {
     if (!LimitSwitch3.get() || !LimitSwitch4.get()) {
       Travelator.set(ControlMode.PercentOutput, 0);
     }
     else {
      Travelator.set(ControlMode.PercentOutput, -opConstants.kTravelatorSpeed);
     }
  }

  public void Stop() {
    Travelator.set(ControlMode.PercentOutput, 0);
  }

  public double getTravelatorPos() {
    return Travelator.getSelectedSensorPosition()/(2048*opConstants.kTravelatorGearRatio/0.12192);
  }
  
}

