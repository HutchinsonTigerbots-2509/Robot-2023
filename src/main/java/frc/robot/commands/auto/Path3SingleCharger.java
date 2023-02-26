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
public class Path3SingleCharger extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  private Drivetrain sDrivetrain;

  /** Creates a new LeftSingleCharger. */
  public Path3SingleCharger(Drivetrain sDrivetrain) {
    this.sDrivetrain = sDrivetrain;

    // blueCommandSequence = Commands.sequence(
    //   sDrivetrain.setCurrentPose(new Pose2d(1.73, 5.03, sDrivetrain.navx.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(6.5,4.57, sDrivetrain.navx.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(1.73,5.03, sDrivetrain.navx.getRotation2d())));
    // redCommandSequence = Commands.sequence( 
    //   sDrivetrain.setCurrentPose(new Pose2d(14.72, 5.03, sDrivetrain.navx.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(10.23,4.57, sDrivetrain.navx.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(14.84,5.03, sDrivetrain.navx.getRotation2d())));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blueCommandSequence = Commands.sequence(
      sDrivetrain.setCurrentPose(new Pose2d(1.73, 5.03, new Rotation2d(180))),
    new DriveToPosition(sDrivetrain, new Pose2d(6,4.57, new Rotation2d(180))),
    new DriveToPosition(sDrivetrain, new Pose2d(1.73,5.03, new Rotation2d(180))));
    redCommandSequence = Commands.sequence( 
      sDrivetrain.setCurrentPose(new Pose2d(14.72, 5.03, sDrivetrain.getRotation2d())),
    new DriveToPosition(sDrivetrain, new Pose2d(10.23,4.57, sDrivetrain.getRotation2d())),
    new DriveToPosition(sDrivetrain, new Pose2d(14.84,5.03, sDrivetrain.getRotation2d())));

    // blueCommandSequence = Commands.sequence(
    //   sDrivetrain.setCurrentPose(new Pose2d(5.03, 1.73, sDrivetrain.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(4.57,6.5, sDrivetrain.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(5.03,1.73, sDrivetrain.getRotation2d())));
    // redCommandSequence = Commands.sequence( 
    //   sDrivetrain.setCurrentPose(new Pose2d(14.72, 5.03, sDrivetrain.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(10.23,4.57, sDrivetrain.getRotation2d())),
    // new DriveToPosition(sDrivetrain, new Pose2d(14.84,5.03, sDrivetrain.getRotation2d())));

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
 // Drop sequence 3

// Move grabber to oposite side

// Drive forward and strafe left

// Grab cube

// Move grabber to drop position

// Drive back

// Drop cube
