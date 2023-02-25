// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Elbow extends SubsystemBase {

  public WPI_TalonSRX armElbow = new WPI_TalonSRX(opConstants.kArmElbowID);
  // public Encoder ElbowEncoder = new Encoder(6, 7);
  private Encoder ElbowEncoder =
      new Encoder(
          opConstants.kElbowEncoder1ID,
          opConstants.kElbowEncoder2ID,
          false,
          Encoder.EncodingType.k4X);

  /** Creates a new Elbow. */
  public Elbow() {
    ElbowEncoder.setDistancePerPulse(4.4);
    ElbowEncoder.setMinRate(1);
    ElbowEncoder.setReverseDirection(false);
    ElbowEncoder.setSamplesToAverage(5);
    ElbowEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElbowEncoder", ElbowEncoder.getDistance());
  }

  /** The Elbow */

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
    return ElbowEncoder.getDistance();
  }
}
