// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Elbow extends SubsystemBase {

  public WPI_TalonFX armElbow = new WPI_TalonFX(opConstants.kArmElbowID);
  // public Encoder ElbowEncoder = new Encoder(6, 7);

  /** Creates a new Elbow. */
  public Elbow() {
    armElbow.setSelectedSensorPosition(0);
    armElbow.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elbow", getElbowPose());
    SmartDashboard.updateValues();
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
    armElbow.set(-opConstants.kElbowSpeed);
  }

  // Command to use elbow forward function
  public Command cmdArmElbowForward() {
    return this.runEnd(this::armElbowForward, this::armElbowStop);
  }

  // Moves the Elbow backwards
  public void armElbowBackward() {
    armElbow.set(opConstants.kElbowSpeed);
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
    armElbow.set(ControlMode.PercentOutput, Speed);
  }

  // Gets the position for the Elbow to move
  public double getElbowPose() {
    return (armElbow.getSelectedSensorPosition() / (2048 * opConstants.kElbowGearRatio / 360))
        - opConstants.kElbowOffSet;
  }
}
