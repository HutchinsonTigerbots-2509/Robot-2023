// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Travelator extends SubsystemBase {
  /** Creates a new Travelator. */

  // ***** Create Motors ***** //
  public VictorSP Travelator = new VictorSP(opConstants.kTravelatorID);

  public Travelator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Moveforward() {
    Travelator.set(opConstants.kTravelatorSpeed);
  }

  public void MoveBackward() {
    Travelator.set(-opConstants.kTravelatorSpeed);
  }

  public void Stop() {
    Travelator.set(0);
  }
}
