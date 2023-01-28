// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.opConstants;
import frc.robot.Constants.ctrlConstants;

public class Drivetrain extends SubsystemBase {

  // ***** Making Variables ***** //
  public double Strafe;
  public double z;

  // ***** Setting up Motors ***** //
  public WPI_TalonFX frontRightMotor = new WPI_TalonFX(opConstants.kFrontRightID);
  public WPI_TalonFX frontLeftMotor = new WPI_TalonFX(opConstants.kFrontLeftID);
  public WPI_TalonFX rearRightMotor = new WPI_TalonFX(opConstants.kRearRightID);
  public WPI_TalonFX rearLeftMotor = new WPI_TalonFX(opConstants.kRearLeftID);

  //Putting all the motors into a Drivetrain
  public MecanumDrive drivetrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  //Setting the start gear to highgear
  private double speedValue = opConstants.kHighSpeed;
  private double speedValueStrafe = opConstants.kHighSpeedStrafe;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    // ***** Inverts all the motors that need to be inverted ***** //
    frontRightMotor.setInverted(false);
    frontLeftMotor.setInverted(true);
    rearRightMotor.setInverted(false);
    rearLeftMotor.setInverted(true);

    // ***** Sets the motors to break when at 0 ***** //
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    rearRightMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftMotor.setNeutralMode(NeutralMode.Brake);

    GearUp();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Runs the Drivetrain with driveCartesian with the values of the stick on the controller */
  public void MecDrive(Joystick stick) {
    drivetrain.driveCartesian(
      -stick.getRawAxis(ctrlConstants.kXboxLeftJoystickY) * speedValue,
      stick.getRawAxis(ctrlConstants.kXboxRightJoystickX) * speedValue,
      stick.getRawAxis(ctrlConstants.kXboxLeftJoystickX) * speedValue
      );
  }
  
  public void TeleMecDrive(double y, double x, double z) {
    drivetrain.driveCartesian(
      y * speedValue,
      x * speedValueStrafe,
      z * speedValue
      );
  }

  /** Drives the autonomous with the speed put into the AutoDrive */
  public void AutoDrive(double xSpeed, double ySpeed, double zSpeed) {
    drivetrain.driveCartesian(xSpeed, ySpeed, zSpeed);
  }

  /** Puts the gear down to be able to slow down driving */
  public void GearDown() {
    speedValue = opConstants.kLowSpeed;
    return;
  }
  
  /** Puts the gear up to be able speed up driving */
  public void GearUp() {
    speedValue = opConstants.kHighSpeed;
    return;
  }

  /** Toggles the gear */
  public void Gear() {
    if (speedValue > .5) {
      speedValue = opConstants.kLowSpeed;
      speedValueStrafe = opConstants.kLowSpeedStrafe;
    }
    else {
      speedValue = opConstants.kHighSpeed;
      speedValueStrafe = opConstants.kHighSpeedStrafe;
    }
    return;
  }

  public double GetSpeedValue() {
    return 0;
  }

  public double GetStrafeValue(Joystick XboxController) {
    if (XboxController.getRawAxis(3) > 0) {
      Strafe = XboxController.getRawAxis(3);
    }
    else if (XboxController.getRawAxis(2) > 0) {
      Strafe = -XboxController.getRawAxis(2);
    }
    else {
      Strafe = 0;
    }
    return Strafe;
  }

}
