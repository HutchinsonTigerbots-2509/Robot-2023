// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPoses;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.Wrist.WristMoveToPosition;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropLowPosition extends ParallelCommandGroup {
  /** Creates a new DropLowPosition. */

  private Dislocator dislocator;
  private Elbow elbow;
  private Shoulder shoulder;
  private Wrist wrist;

  public DropLowPosition(Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder, Wrist pWrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new DislocatorMoveToPosition(0, dislocator),
    new ShoulderMoveToPosition(-200, shoulder),
    new ElbowMoveToPosition(13, elbow),
    new WristMoveToPosition(14, wrist)
    );
  }
}