// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.camConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLight extends SubsystemBase {
  public static double tTurn;
  NetworkTable LimeLightTable = NetworkTableInstance.getDefault().getTable("limelight");


  /** Creates a new LimeLight. */
  public LimeLight() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTargetX() {
    NetworkTableEntry mTableX = LimeLightTable.getEntry(camConstants.kLimelightTargetXID);
    double mTargetX = mTableX.getDouble(1.0);
    return mTargetX;
  }
}
