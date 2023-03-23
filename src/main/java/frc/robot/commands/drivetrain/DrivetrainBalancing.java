// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainBalancing extends CommandBase {
  private final Drivetrain Dt;
  private double Y;
  private double X;
  private double Z;
  private XboxController OpController;
  private Boolean Auto;

  /** Creates a new DriveTele. */
  public DrivetrainBalancing(XboxController pController, Drivetrain pDt) {
    Dt = pDt;
    OpController = pController;
    Auto = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  /** Creates a new DriveTele. */
  public DrivetrainBalancing(Drivetrain pDt, double pX, double pZ) {
    Dt = pDt;
    Auto = true;
    X = pX;
    Z = pZ;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Auto = false) {
      if (Dt.getRoll() < -5) {
        Y = (Dt.getRoll() * .007) - .05;
      } else if (Dt.getRoll() > 5) {
        Y = (Dt.getRoll() * .007) + .05;
      } else {
        Y = 0;
      }

      Dt.mecanumDrive(OpController.getRawAxis(0), -Y, OpController.getRawAxis(4));

    } else {
      if (Dt.getRoll() < -5) {
        Y = (Dt.getRoll() * .007) - .05;
      } else if (Dt.getRoll() > 5) {
        Y = (Dt.getRoll() * .007) + .05;
      } else {
        Y = 0;
      }

      Dt.mecanumDrive(X, -Y, Z);
    }
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
