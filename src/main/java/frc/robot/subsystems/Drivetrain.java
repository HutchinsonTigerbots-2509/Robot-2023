// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ctrlConstants;
import frc.robot.Constants.opConstants;

public class Drivetrain extends SubsystemBase {

 public double Strafe;
  // Nav-X
  AHRS NavX = new AHRS();
  float DisplacementX = NavX.getDisplacementX();
  float DisplacementY = NavX.getDisplacementY();
  float DisplacementZ = NavX.getDisplacementZ();
  float DisplacementRoll = NavX.getRoll();
  float DisplacementPitch = NavX.getPitch();
  float DisplacementYaw = NavX.getYaw();

  // ***** Setting up Motors ***** //
  public WPI_TalonFX frontRightMotor = new WPI_TalonFX(opConstants.kFrontRightID);
  public WPI_TalonFX frontLeftMotor = new WPI_TalonFX(opConstants.kFrontLeftID);
  public WPI_TalonFX rearRightMotor = new WPI_TalonFX(opConstants.kRearRightID);
  public WPI_TalonFX rearLeftMotor = new WPI_TalonFX(opConstants.kRearLeftID);

  //Putting all the motors into a Drivetrain
  public MecanumDrive drivetrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  //Setting the start gear to highgear
  private double speedValue = opConstants.kMaxSpeed;
  private double speedValueStrafe = opConstants.kHighSpeedStrafe;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    this.setName("Drivetrain");
    this.addChild("Mecanum Drive", drivetrain);

    // Inverts all the motors that need to be inverted
    frontRightMotor.setInverted(false);
    frontLeftMotor.setInverted(true);
    rearRightMotor.setInverted(false);
    rearLeftMotor.setInverted(true);

    // Sets the motors to break when stopped
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    rearRightMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftMotor.setNeutralMode(NeutralMode.Brake);

    NavX.calibrate();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void OrientDrive(double y, double x, double z) {
    drivetrain.driveCartesian(y, x, z, NavX.getRotation2d());
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

  /**
   * Returns the current heading of the robot.
   *
   * @return value from -180 to 180 degrees.
   */
  public double getAngle() {
    return NavX.getYaw();
  }

  public void arcadeDrive(double forward, double rotation) {
    drivetrain.driveCartesian(0, forward, rotation);
  }

  public void mecanumDrive(double x, double y, double z, boolean fieldRelative) {
    if (fieldRelative) {
      drivetrain.driveCartesian(x, y, z, Rotation2d.fromDegrees(getAngle()));
    } else {
      drivetrain.driveCartesian(x, y, z, new Rotation2d());
    }
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

  public void stopDrive() {
    drivetrain.stopMotor();
  }

  /**
   * Calculates the distance the wheel has traveled.
   *
   * @param motor
   * @return The distance in meters
   */
  public double getWheelDistance(WPI_TalonFX motor) {
    double rawValue = motor.getSelectedSensorPosition();
    double distance =
        (rawValue / opConstants.kFalconUnitsPerRotation) * opConstants.kWheelDiameter * Math.PI;
    return distance / 100.0;
  }

  /**
   * Constructs a MecanumDriveWheelPosisitons object for the drivetrain.
   *
   * @return MecanumDriveWheelPosisitons
   */
  public MecanumDriveWheelPositions getWheelPositions() {
    double fLeftVal, fRightVal, rLeftVal, rRightVal;

    fLeftVal = getWheelDistance(this.frontLeftMotor);
    fRightVal = getWheelDistance(this.frontRightMotor);
    rLeftVal = getWheelDistance(this.rearLeftMotor);
    rRightVal = getWheelDistance(this.rearRightMotor);

    return new MecanumDriveWheelPositions(fLeftVal, fRightVal, rLeftVal, rRightVal);
  }
}
