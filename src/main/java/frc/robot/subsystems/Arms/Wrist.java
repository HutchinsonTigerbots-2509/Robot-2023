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

public class Wrist extends SubsystemBase {
  public DoubleSolenoid Grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, opConstants.kGrabberP1, opConstants.kGrabberP2);

  public Wrist() {
    Grabber.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Use commands and functions to open and close the grabber

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
}
