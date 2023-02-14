// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Arm extends SubsystemBase {

  // Sets up the Conveyor
  public WPI_TalonFX armMotor1 = new WPI_TalonFX(opConstants.kArmMotor1ID);
  public WPI_VictorSPX armMotor2 = new WPI_VictorSPX(opConstants.kArmMotor2ID);
  public WPI_TalonFX armMotor3 = new WPI_TalonFX(opConstants.kArmMotor3ID);

  private DoubleSolenoid grabber =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, opConstants.kGrabberP1, opConstants.kGrabberP2);

  private Counter normalCounter1 = new Counter();
  private Counter normalCounter2 = new Counter();
  private Counter normalCounter3 = new Counter();

  /** 
   * Creates a new Arm. 
   */
  public Arm() {
    setName("Arm");
    addChild("Grabber", grabber);
    addChild("Motor 1", armMotor1);
    addChild("Motor 2", armMotor2);
    addChild("Motor 3", armMotor3);
    addChild("Count 1", normalCounter1);
    addChild("Count 2", normalCounter2);
    addChild("Count 3", normalCounter3);

    grabber.set(Value.kForward);

    armMotor1.setNeutralMode(NeutralMode.Brake);

    // normalCounter1.setUpSource(opConstants.kArmCounterID);
    // normalCounter1.setUpDownCounterMode();
    // normalCounter1.setMaxPeriod(.1);
    // normalCounter1.setUpdateWhenEmpty(true);
    // normalCounter1.setReverseDirection(false);
    // normalCounter1.setSamplesToAverage(10);
    // normalCounter1.setDistancePerPulse(2.0583);
    // normalCounter1.reset();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.updateValues();
  }

  /** Moves the arm to the pickup position on the cone */
  public void armMoveFinal(int dPos1, int dPos2, int dPos3, int speedValue) {
    armMotor1.set((dPos1 - normalCounter1.get()) * speedValue);
    // armMotor2.set((dPos2 - normalCounter2.get()) * speedValue);
    armMotor3.set((dPos3 - normalCounter3.get()) * speedValue);
  }

  /** Toggles the grabber. */
  public void grab() {
    grabber.toggle();
  }

  /** Runs the arm forward. */
  public void arm1In() {
    armMotor1.set(-opConstants.kMaxArm1Speed);
  }

  /** Runs the arm forward */
  public void arm2In() {
    armMotor2.set(ControlMode.PercentOutput, -opConstants.kMaxArm2Speed);
  }

  /** Runs the arm backwards */
  public void arm1Out() {
    armMotor1.set(opConstants.kMaxArm1Speed);
  }

  /** Runs the arm backwards */
  public void arm2Out() {
    armMotor2.set(ControlMode.PercentOutput, opConstants.kMaxArm2Speed);
  }

  /** Stops the Arm 1 */
  public void arm1Stop() {
    armMotor1.set(0);
  }

  /** Stops the Arm 2 */
  public void arm2Stop() {
    armMotor2.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Running this {@link Command} will toggle the gripper from open to close or close to open.
   *
   * @return {@link Command} to toggle gripper.
   */
  public Command toggleGrip() {
    return this.runOnce(grabber::toggle);
  }

  /**
   * Running this {@link Command} will run Arm 1 up.
   *
   * @return {@link Command} Arm 1 Up
   */
  public Command arm1Up() {
    return this.runEnd(this::arm1In, this::arm1Stop);
  }

  /**
   * Running this {@link Command} will run Arm 1 down.
   *
   * @return {@link Command} Arm 1 Down
   */
  public Command arm1Down() {
    return this.runEnd(this::arm1Out, this::arm1Stop);
  }

  /**
   * Running this {@link Command} will run Arm 2 up.
   *
   * @return {@link Command} Arm 1 Up
   */
  public Command arm2Up() {
    return this.runEnd(this::arm2In, this::arm2Stop);
  }

  /**
   * Running this {@link Command} will run Arm 2 down.
   *
   * @return {@link Command} Arm 1 Down
   */
  public Command arm2Down() {
    return this.runEnd(this::arm2Out, this::arm2Stop);
  }
}
