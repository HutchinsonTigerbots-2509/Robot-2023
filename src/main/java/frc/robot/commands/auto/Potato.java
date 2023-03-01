// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Potato extends InstantCommand {
  private Command commandSequence;

  /** Creates a new LeftSingleCharger. */
  public Potato(Drivetrain sDrivetrain) {
    commandSequence =
        Commands.sequence(
            sDrivetrain.setCurrentPose(new Pose2d(0, 0, sDrivetrain.getRotation2d())),
            new DriveToPosition(sDrivetrain, new Pose2d(2, 2, sDrivetrain.getRotation2d())));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandSequence.schedule();
  }
}
