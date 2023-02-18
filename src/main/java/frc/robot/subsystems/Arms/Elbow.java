// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Elbow extends SubsystemBase {

  public WPI_TalonSRX armElbow = new WPI_TalonSRX(opConstants.kArmElbowID);

  /** Creates a new Elbow. */
  public Elbow() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /** The Elbow */
  public void armElbowForward() {
    armElbow.set(opConstants.kMaxAngularSpeed);
  }

  public Command cmdArmElbowForward() {
    return this.runEnd(this::armElbowForward, this::armElbowStop);
  }

  public void armElbowBackward() {
    armElbow.set(-opConstants.kMaxAngularSpeed);
  }

  public Command cmdArmElbowBackward() {
    return this.runEnd(this::armElbowBackward, this::armElbowStop);
  }

  public void armElbowStop() {
    armElbow.set(0);
  }
}
