// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.camConstants;

public class LimeLight extends SubsystemBase {
  public static double tTurn;
  NetworkTable LimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-one");
  NetworkTable LimeLightTwoTable = NetworkTableInstance.getDefault().getTable("limelight-two");

  /** Creates a new LimeLight. */
  public LimeLight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTargetX() {
    NetworkTableEntry mTableX = LimeLightTable.getEntry(camConstants.kLimelightTargetXID);
    double mTargetX = mTableX.getDouble(0);
    return mTargetX;
  }

  public double getTargetY() {
    NetworkTableEntry mTableY = LimeLightTable.getEntry(camConstants.kLimelightTargetYID);
    double mTargetY = mTableY.getDouble(0);
    return mTargetY;
  }
}
