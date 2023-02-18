// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arms;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Shoulder extends SubsystemBase {

  // Sets up the arm
  public WPI_TalonFX armLift = new WPI_TalonFX(opConstants.kArmLiftID);

  public Counter normalCounter1 = new Counter();
  public Counter normalCounter2 = new Counter();
  public Counter normalCounter3 = new Counter();

  private Encoder JawEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

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

    JawEncoder.setDistancePerPulse(4.4);
    JawEncoder.setMinRate(1);
    JawEncoder.setReverseDirection(false);
    JawEncoder.setSamplesToAverage(5);
    JawEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Counter", normalCounter1.get());
    SmartDashboard.putNumber("Degrees", normalCounter1.getDistance());
    SmartDashboard.putNumber("Jaw Get", JawEncoder.getDistance());
    SmartDashboard.updateValues();
    // System.out.println("\n\nCounter");
    // System.out.println(normalCounter.get());
    // System.out.println(normalCounter.getDistance());
  }

  // public Command grabExtend() {
  //   return this.runOnce(() -> Grabber.set(Value.kForward));
  // }

  //  public Command grabRetract() {
  //   return this.runOnce(() -> Grabber.set(Value.kReverse));
  // }

  /** The tower/lift */
  public void armLiftForward() {
    // Runs the arm
    armLift.set(-opConstants.kMaxArmSpeed);
  }

  public Command cmdArmLiftForward() {
    return this.runEnd(this::armLiftForward, this::armLiftStop);
  }

  public void armLiftBackward() {
    // Runs the arm backwards
    armLift.set(opConstants.kMaxArmSpeed);
  }

  public Command cmdArmLiftBackward() {
    return this.runEnd(this::armLiftBackward, this::armLiftStop);
  }

  public void armLiftStop() {
    // Shops the Arm
    armLift.set(0);
  }

  /** The wrist */
}
