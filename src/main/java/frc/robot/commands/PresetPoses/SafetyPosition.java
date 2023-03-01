// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPoses;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class SafetyPosition extends InstantCommand {

private Dislocator dislocator;
private Elbow elbow;
private Shoulder shoulder;
private Wrist wrist;

  public SafetyPosition(Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder, Wrist pWrist) {
    this.dislocator = pDislocator;
    this.elbow = pElbow;
    this.shoulder = pShoulder;
    this.wrist = pWrist;


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new DislocatorMoveToPosition(0, dislocator);
    new ShoulderMoveToPosition(-200, shoulder);
    new ElbowMoveToPosition(154, elbow);
    new WristMoveToPosition(0, wrist);
  }
}