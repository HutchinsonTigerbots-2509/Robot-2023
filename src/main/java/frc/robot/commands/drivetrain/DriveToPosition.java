// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.opConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPosition extends CommandBase {
  static final double kPx = 0.2;
  static final double kIx = 0.001;
  static final double kDx = 0.00;
  static final double kPy = 0.05;
  static final double kIy = 0.0005;
  static final double kDy = 0.00;
  static final double minSpeed = 0.15;

  SlewRateLimiter xspeedLimiter;
  SlewRateLimiter yspeedLimiter;
  PIDController xController;
  PIDController yController;
  Drivetrain drive;
  Pose2d targetPose, currentPose;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(Drivetrain sDrive, Pose2d targetPose) {
    this.drive = sDrive;
    this.targetPose = targetPose;
    this.xController = new PIDController(kPx, kIx, kDx);
    this.yController = new PIDController(kPy, kIy, kDy);
    this.xController.setTolerance(0.05);
    this.yController.setTolerance(0.05);
    this.xspeedLimiter = new SlewRateLimiter(0.5);//opConstants.kSkewRateLimit);
    this.yspeedLimiter = new SlewRateLimiter(0.5);//opConstants.kSkewRateLimit);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.xController.setSetpoint(targetPose.getX());
    this.yController.setSetpoint(targetPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed, ySpeed;
    this.currentPose = this.drive.getCurrentPose();

    // Raw PID value
    xSpeed = xController.calculate(this.currentPose.getX());
    ySpeed = yController.calculate(this.currentPose.getY());

    if (Math.abs(targetPose.getX() - currentPose.getX()) < 0.05) {
      xSpeed = 0;
    } else {
      // double isNeg = Math.abs(xSpeed) / xSpeed;
      // xSpeed = Math.abs(Math.max(Math.abs(xSpeed), minSpeed));
      // xSpeed *= isNeg;
      if(xSpeed > -minSpeed&& xSpeed <0){
        xSpeed = -minSpeed;
      }else if (xSpeed < minSpeed && xSpeed >0){
        xSpeed = minSpeed;
      }
    }
    if (Math.abs(targetPose.getY() - currentPose.getY()) < 0.05) {
      ySpeed = 0;
    } else {
      // double isNeg = Math.abs(ySpeed) / ySpeed;
      // ySpeed = Math.abs(Math.max(Math.abs(ySpeed), minSpeed));
      // ySpeed *= isNeg;
      if(ySpeed > -minSpeed&&ySpeed<0){
        ySpeed = -minSpeed;
      }else if (ySpeed < minSpeed&&ySpeed>0){
        ySpeed = minSpeed;
      }
    }
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    // Filter through SkewRateLimiter
    xSpeed = xspeedLimiter.calculate(xSpeed) * opConstants.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(ySpeed) * opConstants.kMaxSpeed;
    
    drive.mecanumDrive(-xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(targetPose.getX() - currentPose.getX()) < 0.05
        && Math.abs(targetPose.getY() - currentPose.getY()) < 0.05) {
      return true;
    } else {
      return false;
    }
  }
}
