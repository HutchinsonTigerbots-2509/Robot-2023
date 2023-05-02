// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAutoGyro extends CommandBase {

  private final Drivetrain Dt;
  private double Y;
  private double X;
  private double Z;
  private double ZPos;

  
  private PIDController rotController;

  static final double kP = 0.02;
  static final double kI = 0.001;
  static final double kD = 0.00;
  static final double minSpeed = 0.12;
  

  /** Creates a new DriveTele. */
  public DriveAutoGyro(Drivetrain pDt, double pX, double pY, double pZPos) {
    Dt = pDt;
    X = pX;
    Y = pY;
    ZPos = pZPos;

    this.rotController = new PIDController(kP, kI, kD);
    this.rotController.setTolerance(2);
    this.rotController.enableContinuousInput(-180, 180);
    this.rotController.setSetpoint(ZPos);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  public DriveAutoGyro(Drivetrain pDt, double pY, double pZPos) {
    Dt = pDt;
    X = 0;
    Y = pY;
    ZPos = pZPos;

    this.rotController = new PIDController(kP, kI, kD);
    this.rotController.setTolerance(2);
    this.rotController.enableContinuousInput(-180, 180);
    this.rotController.setSetpoint(ZPos);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotSpeed;
    rotSpeed = rotController.calculate(Dt.getAngle());

    Dt.mecanumDrive(X, -Y, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dt.mecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
