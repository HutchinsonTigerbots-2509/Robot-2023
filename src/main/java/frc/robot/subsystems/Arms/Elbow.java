// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Elbow extends SubsystemBase {

  public WPI_TalonFX armElbow = new WPI_TalonFX(opConstants.kArmElbowID);
  public DigitalInput LimitSwitch5 = new DigitalInput(opConstants.kElbowLimitSwitchTopID);
  public DigitalInput LimitSwitch6 = new DigitalInput(opConstants.kElbowLimitSwitchBottomID);

  // public Encoder ElbowEncoder = new Encoder(6, 7);

  /** Creates a new Elbow. */
  public Elbow() {
    armElbow.setSelectedSensorPosition(
        opConstants.kElbowMin * (2048 * opConstants.kElbowGearRatio / 360));
    armElbow.setInverted(false);
    armElbow.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elbow", getElbowPose());
    SmartDashboard.putNumber("ElbowDirectEncoder", armElbow.getSelectedSensorPosition());
    SmartDashboard.putData("Limit 5", LimitSwitch5);
    SmartDashboard.putData("Limit 6", LimitSwitch6);
    SmartDashboard.updateValues();

    if (!LimitSwitch6.get()) {
      armElbow.setSelectedSensorPosition(
          opConstants.kElbowMax * (2048 * opConstants.kElbowGearRatio / 360));
    }

    if (!LimitSwitch5.get()) {
      armElbow.setSelectedSensorPosition(
          opConstants.kElbowMin * (2048 * opConstants.kElbowGearRatio / 360));
    }
  }

  /** The Elbow */
  public void ResetElbowEncoder() {
    armElbow.setSelectedSensorPosition(0);
  }

  public Command cmdResetElbowEncoder() {
    return this.runOnce(this::ResetElbowEncoder);
  }

  // Moves the Elbow forward
  public void armElbowForward() {
    if (!LimitSwitch5.get()) {
      armElbow.set(ControlMode.PercentOutput, 0);
    } else {
      armElbow.set(ControlMode.PercentOutput, -opConstants.kElbowSpeed);
    }
  }

  // Command to use elbow forward function
  public Command cmdArmElbowForward() {
    return this.runEnd(this::armElbowForward, this::armElbowStop);
  }

  // Moves the Elbow backwards
  public void armElbowBackward() {
    if (!LimitSwitch6.get()) {
      armElbow.set(ControlMode.PercentOutput, 0);
    } else {
      armElbow.set(ControlMode.PercentOutput, opConstants.kElbowSpeed);
    }
  }

  // Command to use the Elbow backward function
  public Command cmdArmElbowBackward() {
    return this.runEnd(this::armElbowBackward, this::armElbowStop);
  }

  // Stops the Elbow after movement
  public void armElbowStop() {
    armElbow.set(0);
  }

  // Moves the Elbow with the comands for going to a position
  public void ElbowMove(double Speed) {
    if (Speed > 0 && (!LimitSwitch6.get())) {
      armElbow.set(ControlMode.PercentOutput, 0);
    } else if (Speed < 0 && (!LimitSwitch5.get())) {
      armElbow.set(ControlMode.PercentOutput, 0);
    } else {
      armElbow.set(ControlMode.PercentOutput, Speed);
      ;
    }
  }

  // Gets the position for the Elbow to move
  public double getElbowPose() {
    return (armElbow.getSelectedSensorPosition() / (2048 * opConstants.kElbowGearRatio / 360));
  }
}
