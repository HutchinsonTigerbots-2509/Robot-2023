// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPoses;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabBackPosition extends ParallelCommandGroup {

  private Dislocator dislocator;
  private Elbow elbow;
  private Shoulder shoulder;
  private Travelator travelator;

  public GrabBackPosition(
      Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder, Travelator pTravelator) {
    this.dislocator = pDislocator;
    this.elbow = pElbow;
    this.shoulder = pShoulder;
    this.travelator = pTravelator;

    // Use addRequirements() here to declare subsystem dependencies.
    this.addCommands(
        new WaitCommand(0.5).andThen(new DislocatorMoveToPosition(4.7, dislocator)),
        new WaitCommand(0.5).andThen(new ShoulderMoveToPosition(153, shoulder)),
        new WaitCommand(0.5).andThen(new ElbowMoveToPosition(-47, elbow)),
        new TravelatorMoveToPosition(0, travelator));
  }
}
