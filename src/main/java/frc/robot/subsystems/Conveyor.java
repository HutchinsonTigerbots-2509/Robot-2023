// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.opConstants;
import frc.robot.RobotContainer;

public class Conveyor extends SubsystemBase {

  //Sets up the Conveyor
  public VictorSP conveyorMotor = new VictorSP(opConstants.kConveyorMotorID);
  public Counter normalCounter = new Counter();

  /** Creates a new Conveyor. */
  public Conveyor() {

    normalCounter.setUpSource(opConstants.kArmCounterID);
    normalCounter.setUpDownCounterMode();
    normalCounter.setMaxPeriod(.1);
    normalCounter.setUpdateWhenEmpty(true);
    normalCounter.setReverseDirection(false);
    normalCounter.setSamplesToAverage(10);
    normalCounter.setDistancePerPulse(2.0583);
    normalCounter.reset();
  }

  @Override
  public void periodic() {
         // This method will be called once per scheduler run
    SmartDashboard.putNumber("Counter", normalCounter.get());
    SmartDashboard.putNumber("Degrees", normalCounter.getDistance());
    SmartDashboard.updateValues();
    // System.out.println("\n\nCounter");
    // System.out.println(normalCounter.get());
    // System.out.println(normalCounter.getDistance());
  }

  public void ConveyorIn() {

    //Runs the Conveyor to be able to pull in balls into the shooter
    conveyorMotor.set(-opConstants.kMaxConveyorSpeed);
  }

  public void ConveyorStop() {

    //Shops the Conveyor
    conveyorMotor.set(0);
  }

  public void ConveyorReverse() {

    //Runs the Conveyor away from the shooter incase the ball gets stuck
    conveyorMotor.set(opConstants.kMaxConveyorSpeed);
  }
}
