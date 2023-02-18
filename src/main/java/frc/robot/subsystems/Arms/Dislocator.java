// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Dislocator extends SubsystemBase {
  /** Creates a new Dislocator. */
  public TalonFX Dislocator = new TalonFX(opConstants.kDislocatorID);

  public Dislocator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DislocatorForward() {}

  public Command cmdDislocatorMoveForward() {
    return this.runEnd(this::DislocatorForward, this::DislocatorStop);
  }

  public void DislocatorBackward() {
    Dislocator.set(-opConstants.kDislocatorSpeed);
  }

  public Command cmdDislocatorMoveBackward() {
    return this.runEnd(this::DislocatorBackward, this::DislocatorStop);
  }

  public void DislocatorStop() {
    Dislocator.set(0);
  }
}
