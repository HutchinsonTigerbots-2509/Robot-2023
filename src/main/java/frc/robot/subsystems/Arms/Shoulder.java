// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;
import frc.robot.commands.Shoulder.ShoulderMoveToPosition;

public class Shoulder extends SubsystemBase {

  // Sets up the arm
  public WPI_TalonFX Shoulder = new WPI_TalonFX(opConstants.kShoulderID);

  Shoulder sShoulder;

  double position = 0;

  /** Creates a new arm. * */
  public Shoulder() {
    Shoulder.setNeutralMode(NeutralMode.Brake);

    //Shoulder.setSelectedSensorPosition(-35 * (2048 * opConstants.kShoulderGearRatio / 360));
    Shoulder.setSelectedSensorPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ShoulderPos", getShoulderPos());
    SmartDashboard.putNumber("DesireShoulderPos", getShoulderDesirePos());
    SmartDashboard.updateValues();

    //new ShoulderMoveToPosition(getShoulderDesirePos(), sShoulder);
    
  }

  /** The tower/lift */

  // Moves Shoulder Forward
  public void ShoulderForward() {
    Shoulder.set(-opConstants.kShoulderSpeed);
  }

  // Command to move the shoulder forward function
  public Command cmdShoulderForward() {
    return this.runEnd(this::ShoulderForward, this::ShoulderStop);
  }

  // Moves Shoulder backward
  public void ShoulderBackward() {
    Shoulder.set(opConstants.kShoulderSpeed);
  }

  // Command to move shoulder backward function
  public Command cmdShoulderBackward() {
    return this.runEnd(this::ShoulderBackward, this::ShoulderStop);
  }

  // Stops the arm after movement
  public void ShoulderStop() {
    Shoulder.set(0);
  }

  public void ShoulderPoseForward() {
    if (position < 200){
      position += .1;
    }
  }

  // Command to move the shoulder forward function
  public Command cmdShoulderPoseForward() {
    return this.run(this::ShoulderPoseForward);
  }

  public void ShoulderPoseBackward() {
    if (position > -200){
      position -= .1;
    }
  }

  // Command to move the shoulder backward function
  public Command cmdShoulderPoseBackward() {
    return this.run(this::ShoulderPoseBackward);
  }

  // Moves the shoulder so that it will move to a position
  public void ShoulderMove(Double Speed) {
    Shoulder.set(Speed);
  }

  public void ShoulderPeriodMove() {
    new ShoulderMoveToPosition(position, sShoulder);
  }

  public double getShoulderDesirePos() {
    return position;
  }

  // Gets the position of the shoulder for the move to position
  public double getShoulderPos() {
    return (Shoulder.getSelectedSensorPosition() / (2048 * opConstants.kShoulderGearRatio / 360)) - opConstants.kShoudlerOffSet;
  }
}
