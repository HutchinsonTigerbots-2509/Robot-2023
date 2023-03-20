// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAutoGyro extends CommandBase {
  private final Drivetrain Dt;
  private double Y;
  private double X;
  private double Z;
  private double ZPos;

  /** Creates a new DriveTele. */
  public DriveAutoGyro(Drivetrain pDt, double pX, double pY, double pZPos) {
    Dt = pDt;
    X = pX;
    Y = pY;
    ZPos = pZPos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Dt.getAngle() + ZPos) < -5) {
      Z = ((Dt.getAngle() + ZPos) * .007) - .06;
    } else if ((Dt.getAngle() + ZPos) > 5) {
      Z = ((Dt.getAngle() * .007) + ZPos) + .06;
    } else {
      Z = 0;
    }

    Dt.mecanumDrive(X, -Y, -Z);
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
