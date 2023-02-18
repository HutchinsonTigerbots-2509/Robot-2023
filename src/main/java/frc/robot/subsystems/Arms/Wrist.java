// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Wrist extends SubsystemBase {
  private Encoder JawEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

  /** Creates a new Wrist. */
  public WPI_TalonSRX armWrist = new WPI_TalonSRX(opConstants.kArmWristID);

  public DoubleSolenoid Grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, opConstants.kGrabberP1, opConstants.kGrabberP2);

  public Wrist() {
    Grabber.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command Grab() {
    return this.runOnce(() -> Grabber.toggle());
  }

  public void armWristForward() {
    armWrist.set(opConstants.kMaxAngularSpeed);
  }

  public Command cmdArmWristForward() {
    return this.runEnd(this::armWristForward, this::armWristStop);
  }

  public void armWristBackward() {
    armWrist.set(-opConstants.kMaxAngularSpeed);
  }

  public Command cmdArmWristBackward() {
    return this.runEnd(this::armWristBackward, this::armWristStop);
  }

  public void WristMove(Double Speed) {
    armWrist.set(Speed);
  }

  public void armWristStop() {
    armWrist.set(0);
  }

  public double getWristPose() {
    return JawEncoder.getDistance(); // Return the shaft sensor position
  }
}
