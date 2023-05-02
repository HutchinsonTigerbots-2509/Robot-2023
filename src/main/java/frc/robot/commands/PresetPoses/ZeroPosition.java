// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPoses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroPosition extends InstantCommand {

  private Dislocator dislocator;
  private Elbow elbow;
  private Shoulder shoulder;

  public ZeroPosition(Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder) {
    this.dislocator = pDislocator;
    this.elbow = pElbow;
    this.shoulder = pShoulder;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new DislocatorMoveToPosition(0, dislocator);
    new ShoulderMoveToPosition(0, shoulder);
    new ElbowMoveToPosition(0, elbow);
  }
}
