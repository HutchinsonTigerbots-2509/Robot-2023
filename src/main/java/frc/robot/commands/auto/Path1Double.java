// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.subsystems.Drivetrain;

public class Path1Double extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  public Path1Double(Drivetrain sDrivetrain) {
    blueCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(1.5, 0.5, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(3.0, 0.75, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(6.202, 0.815, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(1.162, 1.0, new Rotation2d())));
    redCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(14.5, 0.5, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(12.5, 0.75, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(10.5, 0.815, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(14.5, 1.0, new Rotation2d())));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      blueCommandSequence.schedule();
    } else {
      redCommandSequence.schedule();
    }
  }
}
