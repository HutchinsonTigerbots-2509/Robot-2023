// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OperatorDrive extends CommandBase {

  XboxController xboxController;
  Drivetrain drive;
  boolean fieldRelative;

  // Slew rate limiters to make joystick inputs more gentle; ex: 1/3 sec from 0 to 1.
  // private final SlewRateLimiter xspeedLimiter;
  // private final SlewRateLimiter yspeedLimiter;
  // private final SlewRateLimiter rotLimiter;

  /** Creates a new OperatorDrive. */
  public OperatorDrive(Drivetrain drive, XboxController xboxController, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xboxController = xboxController;
    this.drive = drive;
    this.fieldRelative = fieldRelative;
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per
    // second
    // this.xspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.yspeedLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);
    // this.rotLimiter = new SlewRateLimiter(opConstants.kSkewRateLimit);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed =
        xboxController.getRawAxis(
            0); // xspeedLimiter.calculate(xboxController.getRawAxis(0)) * opConstants.kMaxSpeed;
    double ySpeed =
        xboxController.getRawAxis(
            1); // yspeedLimiter.calculate(xboxController.getRawAxis(1)) * opConstants.kMaxSpeed;
    double rot =
        xboxController.getRawAxis(4); // rotLimiter.calculate(xboxController.getRawAxis(4)) *
    // opConstants.kMaxAngularSpeed;
    drive.mecanumDrive(xSpeed, ySpeed, rot, drive.getSpeed());
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
