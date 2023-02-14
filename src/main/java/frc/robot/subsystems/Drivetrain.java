// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;
import frc.robot.Constants.ctrlConstants;

public class Drivetrain extends SubsystemBase {

  // ***** Making Variables ***** //
  public double Strafe;
  public double z;

  // ***** Setting up Drivetrain ***** //
  public WPI_TalonFX frontRightMotor = new WPI_TalonFX(opConstants.kFrontRightID);
  public WPI_TalonFX frontLeftMotor = new WPI_TalonFX(opConstants.kFrontLeftID);
  public WPI_TalonFX rearRightMotor = new WPI_TalonFX(opConstants.kRearRightID);
  public WPI_TalonFX rearLeftMotor = new WPI_TalonFX(opConstants.kRearLeftID);
  public MecanumDrive drivetrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  
  // Speed
  private double speedValue = opConstants.kMaxSpeed;
  private double speedValueStrafe = opConstants.kHighSpeedStrafe;

  // ***** NavX ***** //
  private AHRS NavX = new AHRS();
  float DisplacementX = NavX.getDisplacementX();
  float DisplacementY = NavX.getDisplacementY();
  float DisplacementZ = NavX.getDisplacementZ();
  float DisplacementRoll = NavX.getRoll();
  float DisplacementPitch = NavX.getPitch();
  float DisplacementYaw = NavX.getYaw();

   // Kinematics
  // The locations for the wheels must be relative to the center of the robot.
  // Positive x values represent moving toward the front of the robot whereas
  // positive y values represent moving toward the left of the robot.
  private Translation2d frontLeftTranslate = new Translation2d(0.2921, 0.3175);
  private Translation2d frontRightTranslate = new Translation2d(0.2921, -0.3175);
  private Translation2d rearLeftTranslate = new Translation2d(-0.2921, 0.3175);
  private Translation2d rearRightTranslate = new Translation2d(-0.2921, -0.3175);

  /** Solenoids */ 
  private DoubleSolenoid parkingBrake =
  new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      opConstants.kParkingBrakeExtend,
      opConstants.kParkingBrakeRetract);

  // Creating my kinematics object using the wheel locations.
  private MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftTranslate, frontRightTranslate, rearLeftTranslate, rearRightTranslate);
  // Creating my odometry object from the kinematics object and the initial wheel
  // positions. Here, our starting pose is 5 meters along the long end of the
  // field and in
  // the center of the field along the short end, facing the opposing alliance
  // wall.
  private MecanumDriveOdometry odometry;
  private Pose2d robotPose;
  private Field2d field = new Field2d();

  
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

    NavX.calibrate();

     // Setup Odeometry
     robotPose = new Pose2d(0.0, 0.0, new Rotation2d()); // Inital pose of the robot
     odometry =
         new MecanumDriveOdometry(kinematics, NavX.getRotation2d(), getWheelPositions(), robotPose);
     SmartDashboard.putData("Field", field);
     SmartDashboard.putNumber("Odom X", robotPose.getX());
     SmartDashboard.putNumber("Odom Y", robotPose.getY());
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my wheel positions
    MecanumDriveWheelPositions wheelPositions = getWheelPositions();
    // Get the rotation of the robot from the gyro.
    Rotation2d gyroAngle = NavX.getRotation2d();
    // Update the pose
    robotPose = odometry.update(gyroAngle, wheelPositions);

    // Display Telemetry
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Yaw", NavX.getYaw());
    SmartDashboard.putNumber("Roll", NavX.getRoll());
    SmartDashboard.putNumber("Pitch", NavX.getPitch());
    SmartDashboard.putNumber("X", field.getRobotPose().getX());
    SmartDashboard.putNumber("Y", field.getRobotPose().getY());
    SmartDashboard.updateValues();
  }

  private MecanumDriveWheelPositions getWheelPositions() {
    double fLeftVal, fRightVal, rLeftVal, rRightVal;

    fLeftVal = getWheelDistance(this.frontLeftMotor);
    fRightVal = getWheelDistance(this.frontRightMotor);
    rLeftVal = getWheelDistance(this.rearLeftMotor);
    rRightVal = getWheelDistance(this.rearRightMotor);

    return new MecanumDriveWheelPositions(fLeftVal, fRightVal, rLeftVal, rRightVal);
  }

  public double getWheelDistance(WPI_TalonFX motor) {
    double rawValue = motor.getSelectedSensorPosition();
    double distance =
        (rawValue / opConstants.kFalconUnitsPerRotation)
            / opConstants.kGearRatio
            * opConstants.kWheelDiameter
            * Math.PI;
    return distance / 100.0;
  }
  
  public double getAngle() {
    return NavX.getYaw();
  }

  public void mecanumDrive(double x, double y, double z, boolean fieldRelative) {
    if (fieldRelative) {
      drivetrain.driveCartesian(x, y, z, Rotation2d.fromDegrees(getAngle()));
    } else {
      drivetrain.driveCartesian(x, y, z, new Rotation2d());
    }
  }

  public void stopDrive() {
    drivetrain.stopMotor();
  }
  
  public Command setCurrentPose(Pose2d newPose) {
    return this.runOnce(() -> this.setRobotPose(newPose));
  }

  /** Runs the Drivetrain with driveCartesian with the values of the stick on the controller */
  public void MecDrive(Joystick stick) {
    drivetrain.driveCartesian(
      -stick.getRawAxis(ctrlConstants.kXboxLeftJoystickY) * speedValue,
      stick.getRawAxis(ctrlConstants.kXboxRightJoystickX) * speedValue,
      stick.getRawAxis(ctrlConstants.kXboxLeftJoystickX) * speedValue
      );
  }

  public void OrientDrive(double y, double x, double z) {
    drivetrain.driveCartesian(y, x, z, NavX.getRotation2d());
  }

  public Pose2d getCurrentPose() {
    return robotPose;
  }

  public void resetSensors() {
    // Reset encoder postionN
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
    rearLeftMotor.setSelectedSensorPosition(0);
    rearRightMotor.setSelectedSensorPosition(0);

    // Reset Nav-X
    NavX.reset();
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
    speedValue = opConstants.kMaxSpeed;
    return;
  }

  /** Toggles the gear */
  public void Gear() {
    if (speedValue > .5) {
      speedValue = opConstants.kLowSpeed;
      speedValueStrafe = opConstants.kLowSpeedStrafe;
    }
    else {
      speedValue = opConstants.kMaxSpeed;
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

  public Pose2d setRobotPose(Pose2d newPose) {
    robotPose = newPose;
    odometry.resetPosition(this.NavX.getRotation2d(), getWheelPositions(), newPose);
    field.setRobotPose(odometry.getPoseMeters());
    return field.getRobotPose();
  }

  public Command extendParkingBrake() {
    return this.runOnce(() -> parkingBrake.set(Value.kForward));
  }

  public Command retractParkingBrake() {
    return this.runOnce(() -> parkingBrake.set(Value.kReverse));
  }
}
