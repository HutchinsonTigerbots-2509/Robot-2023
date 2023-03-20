// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Dislocator extends SubsystemBase {
  /** Creates a new Dislocator. */
  public WPI_TalonFX Dislocator = new WPI_TalonFX(opConstants.kDislocatorID);

  public DigitalInput FrontLimitSwitch = new DigitalInput(opConstants.kFrontLimitSwitchID);
  public DigitalInput BackLimitSwitch = new DigitalInput(opConstants.kBackLimitSwitchID);

  public Dislocator() {
    Dislocator.setInverted(false);

    Dislocator.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Dislocator", getDislocatorPos());
    SmartDashboard.updateValues();

    if (!FrontLimitSwitch.get()) {
      // Dislocator.setSelectedSensorPosition(22);
      Dislocator.set(ControlMode.PercentOutput, 0);
    }

    if (!BackLimitSwitch.get()) {
      Dislocator.setSelectedSensorPosition(opConstants.kDislocatorMin);
      Dislocator.set(ControlMode.PercentOutput, 0);
    }
  }

  // Makes Extendor / Dislocator go forward
  public void DislocatorForward() {
    if (!FrontLimitSwitch.get()) {
      Dislocator.set(ControlMode.PercentOutput, 0);
    } else {
      Dislocator.set(ControlMode.PercentOutput, opConstants.kDislocatorSpeed);
    }
  }

  // Command to utilize the Dislocator moving forwards
  public Command cmdDislocatorMoveForward() {
    return this.runEnd(this::DislocatorForward, this::DislocatorStop);
  }

  // Makes Extendor / Dislocator go backward
  public void DislocatorBackward() {
    if (!BackLimitSwitch.get()) {
      Dislocator.set(ControlMode.PercentOutput, 0);
    } else {
      Dislocator.set(ControlMode.PercentOutput, -opConstants.kDislocatorSpeed);
    }
  }

  // Command to utilize the Dislocator moving backwards
  public Command cmdDislocatorMoveBackward() {
    return this.runEnd(this::DislocatorBackward, this::DislocatorStop);
  }

  // Stops the Dislocator after movement
  public void DislocatorStop() {
    Dislocator.set(0);
  }

  public double getDislocatorPos() {
    return (Dislocator.getSelectedSensorPosition() / (2048 * 48 / 6.25))
        * 3.66666666666666666; // BS numbers ignore we just needed what we want
  }

  public void DislocatorMove(double Speed) {
    if (Speed > 0 && (!FrontLimitSwitch.get())) {
      Dislocator.set(ControlMode.PercentOutput, 0);
    } else if (Speed < 0 && (!BackLimitSwitch.get())) {
      Dislocator.set(ControlMode.PercentOutput, 0);
    } else {
      Dislocator.set(ControlMode.PercentOutput, Speed);
    }
  }
}
