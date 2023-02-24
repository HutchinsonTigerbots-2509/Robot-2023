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
public class MiddleSingleCharger extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  /** Creates a new LeftSingleCharger. */
  public MiddleSingleCharger(Drivetrain sDrivetrain) {
    blueCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(1.5, 2.5, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(1.947, 2.791, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(3.843, 2.553, new Rotation2d()))
            // end
            );
    redCommandSequence =
        Commands.sequence(
            // This is your starting location on the field. The blue corner is 0,0
            // When choosing target poses, you only need 3 decimal places.
            sDrivetrain.setCurrentPose(new Pose2d(14.75, 2.5, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(14.61, 2.74, new Rotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(12.75, 2.88, new Rotation2d())));

    // end);
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
