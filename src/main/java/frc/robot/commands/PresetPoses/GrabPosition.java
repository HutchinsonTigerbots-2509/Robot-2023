// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PresetPoses;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class GrabPosition extends SequentialCommandGroup {

  private Dislocator dislocator;
  private Elbow elbow;
  private Shoulder shoulder;
  private Travelator travelator;

  public GrabPosition(
      Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder, Travelator pTravelator) {
    this.dislocator = pDislocator;
    this.elbow = pElbow;
    this.shoulder = pShoulder;
    this.travelator = pTravelator;

    // Use addRequirements() here to declare subsystem dependencies.
    this.addCommands(
        Commands.parallel(
            new DislocatorMoveToPosition(22, dislocator),
            new ShoulderMoveToPosition(-118, shoulder),
            new ElbowMoveToPosition(40, elbow)),
        new WaitCommand(1),
        new TravelatorMoveToPosition(2, travelator));
  }
}
