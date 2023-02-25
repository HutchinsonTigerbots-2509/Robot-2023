// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Dislocator extends SubsystemBase {
  /** Creates a new Dislocator. */
  public WPI_TalonFX Dislocator = new WPI_TalonFX(opConstants.kDislocatorID);

  public Dislocator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Makes Extendor / Dislocator go forward
  public void DislocatorForward() {
    Dislocator.set(-opConstants.kDislocatorSpeed);
  }

  // Command to utilize the Dislocator moving forwards
  public Command cmdDislocatorMoveForward() {
    return this.runEnd(this::DislocatorForward, this::DislocatorStop);
  }

  // Makes Extendor / Dislocator go backward
  public void DislocatorBackward() {
    Dislocator.set(opConstants.kDislocatorSpeed);
  }

  // Command to utilize the Dislocator moving backwards
  public Command cmdDislocatorMoveBackward() {
    return this.runEnd(this::DislocatorBackward, this::DislocatorStop);
  }

  // Stops the Dislocator after movement
  public void DislocatorStop() {
    Dislocator.set(0);
  }
}
