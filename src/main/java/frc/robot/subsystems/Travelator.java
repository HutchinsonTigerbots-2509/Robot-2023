// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Travelator extends SubsystemBase {
  /** Creates a new Travelator. */

  // ***** Create Motors ***** //
  public TalonFX Travelator = new TalonFX(opConstants.kTravelatorID);

  // ***** The Limit Switches ***** //
  public DigitalInput LimitSwitch1 = new DigitalInput(opConstants.kBackRightLimitSwitchID);
  public DigitalInput LimitSwitch2 = new DigitalInput(opConstants.kBackLeftLimitSwitchID);
  public DigitalInput LimitSwitch3 = new DigitalInput(opConstants.kFrontRightLimitSwitchID);
  public DigitalInput LimitSwitch4 = new DigitalInput(opConstants.kFrontLeftLimitSwitchID);

  public Travelator() {}

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

  public Command Moveforward() {
    if (!LimitSwitch1.get() || !LimitSwitch2.get()) { // we use not limit switch since the current switches are true until pressed.
      Travelator.set(0);
    } else {
      Travelator.set(opConstants.kTravelatorSpeed);
    };
    return null;
  }

  public Command MoveBackward() {
    if (!LimitSwitch3.get() || !LimitSwitch4.get()) {
      Travelator.set(0);
    } else {
      Travelator.set(-opConstants.kTravelatorSpeed);
    }
    return null;
    }

  public void Stop() {
    Travelator.set(0);
  }

  public Command moveTravelatorMiddle() {
    return null; //this.runOnce(() -> Travelator.);
  }

}
