// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.opConstants;

public class Drivetrain extends SubsystemBase {

  // Setting up Motors
  private WPI_TalonFX frontRightMotor = new WPI_TalonFX(opConstants.kFrontRightID);
  private WPI_TalonFX frontLeftMotor = new WPI_TalonFX(opConstants.kFrontLeftID);
  private WPI_TalonFX rearRightMotor = new WPI_TalonFX(opConstants.kRearRightID);
  private WPI_TalonFX rearLeftMotor = new WPI_TalonFX(opConstants.kRearLeftID);

  private MecanumDrive drivetrain;

  // Nav-X
  private AHRS navx = new AHRS();

  // Parking Brake Cylinders
  private DoubleSolenoid parkingBrake =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          opConstants.kParkingBrakeExtend,
          opConstants.kParkingBrakeRetract);

  // Kinematics
  // The locations for the wheels must be relative to the center of the robot.
  // Positive x values represent moving toward the front of the robot whereas
  // positive y values represent moving toward the left of the robot.
  private Translation2d frontLeftTranslate = new Translation2d(0.2921, 0.3175);
  private Translation2d frontRightTranslate = new Translation2d(0.2921, -0.3175);
  private Translation2d rearLeftTranslate = new Translation2d(-0.2921, 0.3175);
  private Translation2d rearRightTranslate = new Translation2d(-0.2921, -0.3175);

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

    parkingBrake.set(Value.kForward);

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

    // Putting all the motors into a Drivetrain
    drivetrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    // Reset Sensors before defining Odom.
    resetSensors();

    // Setup Odeometry
    robotPose = new Pose2d(0.0, 0.0, new Rotation2d()); // Inital pose of the robot
    odometry =
        new MecanumDriveOdometry(kinematics, getRotation2d(), getWheelPositions(), robotPose);

    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Odom X", robotPose.getX());
    SmartDashboard.putNumber("Odom Y", robotPose.getY());

    this.setName("Drivetrain");
    this.addChild("Mecanum Drive", drivetrain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my wheel positions
    MecanumDriveWheelPositions wheelPositions = getWheelPositions();
    // Get the rotation of the robot from the gyro.
    Rotation2d gyroAngle = getRotation2d();
    // Update the pose
    robotPose = odometry.update(gyroAngle, wheelPositions);
    field.setRobotPose(odometry.getPoseMeters());

    // Display Telemetry
    SmartDashboard.putNumber("Yaw", navx.getYaw());
    SmartDashboard.putNumber("Roll", navx.getRoll());
    SmartDashboard.putNumber("Pitch", navx.getPitch());
    SmartDashboard.putNumber("X", field.getRobotPose().getX());
    SmartDashboard.putNumber("Y", field.getRobotPose().getY());
    SmartDashboard.putNumber("Angle", getRotation2d().getDegrees());
    SmartDashboard.updateValues();
  }

  /**
   * Constructs a MecanumDriveWheelPosisitons object for the drivetrain.
   *
   * @return MecanumDriveWheelPosisitons
   */
  private MecanumDriveWheelPositions getWheelPositions() {
    double fLeftVal;
    double fRightVal;
    double rLeftVal;
    double rRightVal;

    fLeftVal = getWheelDistance(this.frontLeftMotor);
    fRightVal = getWheelDistance(this.frontRightMotor);
    rLeftVal = getWheelDistance(this.rearLeftMotor);
    rRightVal = getWheelDistance(this.rearRightMotor);

    return new MecanumDriveWheelPositions(fLeftVal, fRightVal, rLeftVal, rRightVal);
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
        (rawValue / opConstants.kFalconUnitsPerRotation)
            / opConstants.kGearRatio
            * opConstants.kWheelDiameter
            * Math.PI;
    return distance / 100.0;
  }

  public void resetSensors() {
    // Reset encoder postionN
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
    rearLeftMotor.setSelectedSensorPosition(0);
    rearRightMotor.setSelectedSensorPosition(0);

    // Reset Nav-X
    navx.reset();
  }

  /** Gets the Nav-X */
  public AHRS getNavX() {
    return navx;
  }

  /**
   * Returns the current heading of the robot.
   *
   * @return value from -180 to 180 degrees.
   */
  public double getAngle() {
    return navx.getYaw();
  }

  /**
   * Returns the current rotation and considers if the start on red alliance
   *
   * @return
   */
  public Rotation2d getRotation2d() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return new Rotation2d(navx.getAngle() + 180);
    } else {
      return new Rotation2d(navx.getAngle());
    }
  }

  public void arcadeDrive(double forward, double rotation) {
    drivetrain.driveCartesian(forward, 0, rotation);
  }

  /** Runs the Drivetrain with driveCartesian with the values of the stick on the controller */
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

  public Pose2d getCurrentPose() {
    return robotPose;
  }

  public Pose2d setRobotPose(Pose2d newPose) {
    robotPose = newPose;
    odometry.resetPosition(getRotation2d(), getWheelPositions(), newPose);
    field.setRobotPose(odometry.getPoseMeters());
    return field.getRobotPose();
  }

  public Command setCurrentPose(Pose2d newPose) {
    return this.runOnce(() -> this.setRobotPose(newPose));
  }

  public Command cmdToggleBrake() {
    return this.runOnce(parkingBrake::toggle);
  }

  public double getRoll() {
    return navx.getRoll();
  }
}
