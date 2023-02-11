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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path1Double extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  public Path1Double(Drivetrain sDrivetrain) {
    blueCommandSequence =
        Commands.sequence(
            new DriveToPosition(
                sDrivetrain, new Pose2d(1.944496403743151, 0.477247225100959, new Rotation2d())),
            new DriveToPosition(
                sDrivetrain, new Pose2d(6.202532950021199, 0.815038327521982, new Rotation2d())),
            new DriveToPosition(
                sDrivetrain, new Pose2d(1.16298497890432, 2.030060169629243, new Rotation2d())));
    redCommandSequence =
        Commands.sequence(
            new DriveToPosition(
                sDrivetrain, new Pose2d(14.634718153442892, 0.382807439027479, new Rotation2d())),
            new DriveToPosition(
                sDrivetrain, new Pose2d(10.468843760267244, 0.886324254209018, new Rotation2d())),
            new DriveToPosition(
                sDrivetrain, new Pose2d(14.53695576595974, 1.091239527336539, new Rotation2d())));
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
