// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;


public class Travelator extends SubsystemBase {
  /** Creates a new Travelator. */

  // ***** Create Motors ***** //
  public TalonFX Travelator = new TalonFX(opConstants.kTravelatorID);
  public DigitalInput LimitSwitch1 = new DigitalInput(opConstants.kDetonator1ID);
  public DigitalInput LimitSwitch2 = new DigitalInput(opConstants.kDetonator2ID);

  public Travelator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Switch1", LimitSwitch1);
    SmartDashboard.putData("Switch2", LimitSwitch2);
    SmartDashboard.updateValues();
  }

  public void Moveforward() {
    if (LimitSwitch1.isAnalogTrigger()) {
      Travelator.set(ControlMode.PercentOutput, 0);
    }
    else {
      Travelator.set(ControlMode.PercentOutput, opConstants.kTravelatorSpeed);
    }
  }

  public void MoveBackward() {
    if (LimitSwitch1.isAnalogTrigger()) {
      Travelator.set(ControlMode.PercentOutput, 0);
    }
    else {
      Travelator.set(ControlMode.PercentOutput, -opConstants.kTravelatorSpeed);
    }
  }

  public void Stop() {
    Travelator.set(ControlMode.PercentOutput, 0);
  }
  
  public void getTravelatorPos() {
    Travelator.getSelectedSensorPosition();
  }
  
}

