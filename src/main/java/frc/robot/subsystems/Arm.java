// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Arm extends SubsystemBase {

  // Sets up the Conveyor
  public VictorSP armMotor1 = new VictorSP(opConstants.kArmMotor1ID);
  public VictorSP armMotor2 = new VictorSP(opConstants.kArmMotor2ID);
  public VictorSP armMotor3 = new VictorSP(opConstants.kArmMotor3ID);

  public DoubleSolenoid Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, opConstants.kGrabberP1, opConstants.kGrabberP2);

  public Counter normalCounter1 = new Counter();
  public Counter normalCounter2 = new Counter();
  public Counter normalCounter3 = new Counter();

  /** Creates a new Conveyor. */
  public Arm() {
    Grabber.set(Value.kForward);

    normalCounter1.setUpSource(opConstants.kArmCounterID);
    normalCounter1.setUpDownCounterMode();
    normalCounter1.setMaxPeriod(.1);
    normalCounter1.setUpdateWhenEmpty(true);
    normalCounter1.setReverseDirection(false);
    normalCounter1.setSamplesToAverage(10);
    normalCounter1.setDistancePerPulse(2.0583);
    normalCounter1.reset();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Counter", normalCounter1.get());
    SmartDashboard.putNumber("Degrees", normalCounter1.getDistance());
    SmartDashboard.updateValues();
    // System.out.println("\n\nCounter");
    // System.out.println(normalCounter.get());
    // System.out.println(normalCounter.getDistance());
  }

  //Moves the arm to the pickup position on the cone
  public void ArmMoveFinal(int DPos1, int DPos2, int DPos3, int speedValue) {

    armMotor1.set((DPos1 - normalCounter1.get()) * speedValue);
    armMotor2.set((DPos2 - normalCounter2.get()) * speedValue);
    armMotor3.set((DPos3 - normalCounter3.get()) * speedValue);

  }

  public void Grab() {
    Grabber.toggle();
  }

  public void armIn() {
    //Runs the arm
    armMotor1.set(-opConstants.kMaxArmSpeed);
  }

  public void armOut() {
    //Runs the arm backwards
    armMotor1.set(opConstants.kMaxArmSpeed);
  }

  public void armStop() {
    //Shops the Arm
    armMotor1.set(0);
  }
}