// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAuto extends CommandBase {

  Drivetrain drive;
  Double X;
  Double Y;
  Double Z;
  Double StartingAngle = 0.0;

  // Slew rate limiters to make joystick inputs more gentle; ex: 1/3 sec from 0 to 1.
  // private final SlewRateLimiter xspeedLimiter;
  // private final SlewRateLimiter yspeedLimiter;
  // private final SlewRateLimiter rotLimiter;

  /** Creates a new OperatorDrive. */
  public DriveAuto(Drivetrain drive, double y, double x, double z) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.Y = y;
    this.X = x;
    this.Z = z;
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per
    // second
    // this.xspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.yspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.rotLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);

    addRequirements(drive);
  }

  public DriveAuto(Drivetrain drive, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.Y = y;
    this.X = 0.0;
    this.Z = 0.0;
    this.StartingAngle = drive.getAngle();
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per
    // second
    // this.xspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.yspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.rotLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.StartingAngle = drive.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double zCorr = StartingAngle - drive.getAngle();
    zCorr *= 0.05;

    drive.mecanumDrive(X, -Y, Z + zCorr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
