// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Shoulder extends SubsystemBase {

  // Sets up the arm
  public WPI_TalonFX armLift = new WPI_TalonFX(opConstants.kShoulderID);

  public Counter normalCounter1 = new Counter();
  public Counter normalCounter2 = new Counter();
  public Counter normalCounter3 = new Counter();

  /** Creates a new arm. * */
  public Shoulder() {

    // normalCounter1.setUpSource(opConstants.kArmCounterID);
    // normalCounter1.setUpDownCounterMode();
    // normalCounter1.setMaxPeriod(.1);
    // normalCounter1.setUpdateWhenEmpty(true);
    // normalCounter1.setReverseDirection(false);
    // normalCounter1.setSamplesToAverage(10);
    // normalCounter1.setDistancePerPulse(2.0583);
    // normalCounter1.reset();

    armLift.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Counter", normalCounter1.get());
    SmartDashboard.putNumber("Degrees", normalCounter1.getDistance());
    SmartDashboard.updateValues();
  }

  /** The tower/lift */

  // Moves Shoulder Forward
  public void ShoulderForward() {
    armLift.set(-opConstants.kShoulderSpeed);
  }

  // Command to move the shoulder forward function
  public Command cmdShoulderForward() {
    return this.runEnd(this::ShoulderForward, this::ShoulderStop);
  }

  // Moves Shoulder backward
  public void ShoulderBackward() {
    armLift.set(opConstants.kShoulderSpeed);
  }

  // Command to move shoulder backward function
  public Command cmdShoulderBackward() {
    return this.runEnd(this::ShoulderBackward, this::ShoulderStop);
  }

  // Stops the arm after movement
  public void ShoulderStop() {
    armLift.set(0);
  }

  // Moves the shoulder so that it will move to a position
  public void ShoulderMove(Double Speed) {
    armLift.set(Speed);
  }

  // Gets the position of the shoulder for the move to position
  public double getShoulderPos() {
    return armLift.getSelectedSensorPosition();
  }
}
