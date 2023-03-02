// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainBalancing extends CommandBase {
  private final Drivetrain Dt;
  private double Y;
  private Joystick stick;

  /** Creates a new DriveTele. */
  public DrivetrainBalancing(Joystick pstick, Drivetrain pDt) {
    Dt = pDt;
    stick = pstick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Dt.getRoll() < -2) {
      Y = (Dt.getRoll() * .014) - .15;
    } else if (Dt.getRoll() > 2) {
      Y = (Dt.getRoll() * .014) + .15;
    } else {
      Y = 0;
    }

    Dt.TeleMecDrive(Y, stick.getX(), stick.getZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dt.TeleMecDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
