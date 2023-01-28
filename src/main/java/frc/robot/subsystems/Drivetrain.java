// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
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

  // Nav-X
  public AHRS navx = new AHRS();

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

    navx.reset();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw",navx.getYaw());
    SmartDashboard.updateValues();
  }

  /**
   * Returns the current heading of the robot.
   * @return value from -180 to 180 degrees.
   */
  public double GetAngle(){
    return navx.getYaw();
  }

  public void ArcadeDrive(double forward, double rotation){
    drivetrain.driveCartesian(0, forward, rotation);
  }

  public void MecanumDrive(double x,double y, double z){
    drivetrain.driveCartesian(x, y, z, Rotation2d.fromDegrees(GetAngle()));
  }

  public void StopDrive(){drivetrain.stopMotor();}

  


}
