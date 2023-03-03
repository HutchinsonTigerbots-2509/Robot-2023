// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    addChild("Motor", Travelator);

    addChild("Switch 1", LimitSwitch1);
    addChild("Switch 2", LimitSwitch2);
    addChild("Switch 3", LimitSwitch3);
    addChild("Switch 4", LimitSwitch4);

    Travelator.setNeutralMode(NeutralMode.Brake);
    Travelator.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //  SmartDashboard.putBoolean("TV Back SW", !LimitSwitch1.get()|| !LimitSwitch2.get());
    //  SmartDashboard.putBoolean("TV Front SW", !LimitSwitch3.get()||! LimitSwitch4.get());
    // SmartDashboard.putNumber("TV Ticks", Travelator.getSelectedSensorPosition());
    // SmartDashboard.putNumber("TV Distance", getTravelatorPos());
    //SmartDashboard.putNumber("TravelPos", getTravelatorPos());
    //SmartDashboard.putNumber("TravelSensor", Travelator.getSelectedSensorPosition());
    SmartDashboard.updateValues();

    if (!LimitSwitch1.get() || !LimitSwitch2.get()) {
      Travelator.setSelectedSensorPosition(opConstants.kTravelatorMin);
      Travelator.set(ControlMode.PercentOutput, 0);
    }

    if (!LimitSwitch3.get() || !LimitSwitch4.get()) {
      Travelator.setSelectedSensorPosition(opConstants.kTravelatorMax);
      Travelator.set(ControlMode.PercentOutput, 0);
    }
  }

  public void MoveBackward() {
    if (!LimitSwitch1.get() || !LimitSwitch2.get()) {
      Travelator.set(ControlMode.PercentOutput, 0);
    } else {
      Travelator.set(ControlMode.PercentOutput, -opConstants.kTravelatorSpeed);
    }
  }

  public void MoveForward() {
    if (!LimitSwitch3.get() || !LimitSwitch4.get()) {
      Travelator.set(ControlMode.PercentOutput, 0);
    } else {
      Travelator.set(ControlMode.PercentOutput, opConstants.kTravelatorSpeed);
    }
  }

  public void Move(double Speed) {
    if (Speed < 0 && (!LimitSwitch1.get() || !LimitSwitch2.get())) {
      Travelator.set(ControlMode.PercentOutput, 0);
    } else if (Speed > 0 && (!LimitSwitch3.get() || !LimitSwitch4.get())) {
      Travelator.set(ControlMode.PercentOutput, 0);
    } else {
      Travelator.set(ControlMode.PercentOutput, Speed);
    }
  }

  public void Stop() {
    Travelator.set(ControlMode.PercentOutput, 0);
    Travelator.getSelectedSensorPosition();
  }

  public double getTravelatorPos() {
    return Travelator.getSelectedSensorPosition()
        / (2048 * opConstants.kTravelatorGearRatio / 2.70203);
  }

  /**
   * Move Travelator forwards command
   *
   * @return
   */
  public Command cmdMoveForward() {
    return this.runEnd(this::MoveForward, this::Stop);
  }

  /**
   * Move Travelator backwards command
   *
   * @return
   */
  public Command cmdMoveBackward() {
    return this.runEnd(this::MoveBackward, this::Stop);
  }
}
