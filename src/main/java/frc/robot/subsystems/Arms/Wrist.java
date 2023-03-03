// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;
import frc.robot.commands.Arm.Wrist.WristMoveToPosition;

public class Wrist extends SubsystemBase {
  private Encoder WristEncoder =
      new Encoder(
          opConstants.kWristEncoder1ID,
          opConstants.kWristEncoder2ID,
          false,
          Encoder.EncodingType.k4X);

  /** Creates a new Wrist. */
  public WPI_TalonSRX Wrist = new WPI_TalonSRX(opConstants.kArmWristID);

  public DoubleSolenoid Grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, opConstants.kGrabberP1, opConstants.kGrabberP2);

  public Wrist() {
    Grabber.set(Value.kReverse);

    WristEncoder.setDistancePerPulse(3.63636);
    WristEncoder.setMinRate(1);
    WristEncoder.setReverseDirection(false);
    WristEncoder.setSamplesToAverage(5);
    WristEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WristEncoder", WristEncoder.getDistance());
  }

  // Use commands and functions to open and close the grabber

  public void ResetWristEncoder() {
    WristEncoder.reset();
  }

  // Function to initiate the grabber (claw) opening
  public void GrabOpen() {
    Grabber.set(Value.kForward);
  }

  // Actual command to utilize the function
  public Command cmdGrabOpen() {
    return this.runOnce(this::GrabOpen);
  }

  // Actual command to utilize the function
  public InstantCommand cmdTestGrabOpen() {
    return (InstantCommand) this.runOnce(this::GrabOpen);
  }

  // Function to close the grabber
  public void GrabClose() {
    Grabber.set(Value.kReverse);
  }

  // Command to utilize the grabber closing
  public Command cmdGrabClose() {
    return this.runOnce(this::GrabClose);
  }

  // Command to toggle the grabber (mostly used for reset & preset)
  public Command GrabToggle() {
    return this.runOnce(() -> Grabber.toggle());
  }

  // Commands and functions for the Wrist

  // Moves Wrist Forward
  public void WristForward() {
    Wrist.set(opConstants.kWristSpeed);
  }

  // Command to move the wrist forward function
  public Command cmdWristForward() {
    return this.runEnd(this::WristForward, this::WristStop);
  }

  // Moves Wrist Backward
  public void WristBackward() {
    Wrist.set(-opConstants.kWristSpeed);
  }

  // Command to move the wrist backward function
  public Command cmdWristBackward() {
    return this.runEnd(this::WristBackward, this::WristStop);
  }

  // public void WristWave(Wrist wrist) {
  //   new WristMoveToPosition(-20, wrist);
  //   new WristMoveToPosition(20, wrist);
  // }

  // public Command cmdWristWave(Wrist wrist) {
  //   return this.runEnd(WristWave(wrist), this::WristStop);
  // }

  // Moves wrist to position from the WristToPosition command
  public void WristMove(Double Speed) {
    Wrist.set(Speed);
  }

  // Stops the wrist after movement
  public void WristStop() {
    Wrist.set(0);
  }

  // Fetches the encoder's (placed on the back of the jaw) distance
  public double getWristPose() {
    return WristEncoder.getDistance(); // Return the shaft sensor position
  }
}
