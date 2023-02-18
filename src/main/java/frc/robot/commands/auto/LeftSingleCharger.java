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
public class LeftSingleCharger extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  /** Creates a new LeftSingleCharger. */
  public LeftSingleCharger(Drivetrain sDrivetrain) {

    blueCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(1.5, 5.0, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(1.81, 4.97, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(6.49, 4.57, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(1.76, 4.45, new Rotation2d())));
    redCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(14.75, 5.0, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(14.72, 4.95, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(10.23, 4.65, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(14.84, 4.38, new Rotation2d())));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sDrivetrain);
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
} // Drop sequence 3

// Move grabber to opposite side

// Drive forward and strafe left

// Grab cube

// Stafe left

// Drive back

// Deploy compressor
