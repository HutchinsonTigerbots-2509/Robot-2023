// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Travelator extends SubsystemBase {
  /** Creates a new Travelator. */

  // ***** Create Motors ***** //
  private TalonFX Travelator = new TalonFX(opConstants.kTravelatorID);

  // ***** The Limit Switches ***** //
  private DigitalInput LimitSwitch1 = new DigitalInput(opConstants.kBackRightLimitSwitchID);
  private DigitalInput LimitSwitch2 = new DigitalInput(opConstants.kBackLeftLimitSwitchID);
  private DigitalInput LimitSwitch3 = new DigitalInput(opConstants.kFrontRightLimitSwitchID);
  private DigitalInput LimitSwitch4 = new DigitalInput(opConstants.kFrontLeftLimitSwitchID);

  public Travelator() {
    setName("Travelator");
    addChild("Motor", Travelator);
    addChild("Switch 1", LimitSwitch1);
    addChild("Switch 2", LimitSwitch2);
    addChild("Switch 3", LimitSwitch3);
    addChild("Switch 4", LimitSwitch4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.updateValues();
  }

  public void travelatorPose() {
    if (!LimitSwitch1.get() || !LimitSwitch2.get()) {
      Travelator.getPosition();
    }
  }

  public void moveforward() {
    // we use not limit switch since the current switches are true until pressed.
    if (!LimitSwitch1.get() || !LimitSwitch2.get()) {
      Travelator.set(0);
    } else {
      Travelator.set(opConstants.kTravelatorSpeed);
    }
  }

  public void moveBackward() {
    if (!LimitSwitch3.get() || !LimitSwitch4.get()) {
      Travelator.set(0);
    } else {
      Travelator.set(-opConstants.kTravelatorSpeed);
    }
  }

  public void stop() {
    Travelator.set(0);
  }

  public Command moveTravelatorMiddle() {
    return null; // this.runOnce(() -> Travelator.);
  }

  /**
   * Move Travelator forwards command
   *
   * @return
   */
  public Command moveTravelatorForward() {
    // We use the RunEnd function because moveForward() will
    // be called every iteration.
    return this.runEnd(this::moveforward, this::stop);
  }

  /**
   * Move Travelator backwards command
   *
   * @return
   */
  public Command moveTravelatorBackward() {
    // We use the RunEnd function because moveBackward() will
    // be called every iteration.
    return this.runEnd(this::moveBackward, this::stop);
  }
}
