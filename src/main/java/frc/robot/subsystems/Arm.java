// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Arm extends SubsystemBase {

  // Sets up the arm
  public WPI_TalonFX armLift = new WPI_TalonFX(opConstants.kArmLiftID);
  public WPI_TalonSRX armKnuckle = new WPI_TalonSRX(opConstants.kArmKnuckleID);
  public WPI_TalonSRX armWrist = new WPI_TalonSRX(opConstants.kArmWristID);

  public DoubleSolenoid Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, opConstants.kParkingBrakeP1, opConstants.kParkingBrakeP2);

  public Counter normalCounter1 = new Counter();
  public Counter normalCounter2 = new Counter();
  public Counter normalCounter3 = new Counter();

  /** Creates a new arm. **/
  public Arm() {
   // Grabber.set(Value.kForward);

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
    // System.out.println("\n\nCounter");
    // System.out.println(normalCounter.get());
    // System.out.println(normalCounter.getDistance());
  }

  //Moves the arm to the pickup position on the cone
  public void ArmMoveFinal(int DPos1, int DPos2, int DPos3, int speedValue) {

    armLift.set((DPos1 - normalCounter1.get()) * speedValue);
    armKnuckle.set((DPos2 - normalCounter2.get()) * speedValue);
    armWrist.set((DPos3 - normalCounter3.get()) * speedValue);

  }

  public void Grab() {
    Grabber.toggle();
  }

  /** The tower/lift */
  public void armLiftIn() {
    //Runs the arm
    armLift.set(-opConstants.kMaxArmSpeed);
  }

  public void armLiftOut() {
    //Runs the arm backwards
    armLift.set(opConstants.kMaxArmSpeed);
  }

  public void armLiftStop() {
    //Shops the Arm
    armLift.set(0);
  }


  /** The Knuckle */
  public void armKnuckleIn() {
    armKnuckle.set(opConstants.kMaxAngularSpeed);
  }

  public void armKnuckleOut() {
    armKnuckle.set(-opConstants.kMaxAngularSpeed);
  }

  public void armKnuckleStop() {
    armKnuckle.set(0);
  }

  
  /** The wrist */
  public void armWristIn() {
    armWrist.set(opConstants.kMaxAngularSpeed);
  }

  public void armWristOut() {
    armWrist.set(-opConstants.kMaxAngularSpeed);
  }

  public void armWristStop() {
    armWrist.set(0);
  }
}