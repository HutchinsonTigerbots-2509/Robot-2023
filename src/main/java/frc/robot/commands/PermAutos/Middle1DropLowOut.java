// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PermAutos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.Grabber.GrabOpen;
import frc.robot.commands.PresetPoses.CompressPosition;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.drivetrain.DriveAuto;
import frc.robot.commands.drivetrain.DrivetrainBalancing;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Middle1DropLowOut extends InstantCommand {
  private Command blueCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Wrist wrist;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public Middle1DropLowOut(
      Drivetrain pDrivetrain,
      Dislocator pDislocator,
      Elbow pElbow,
      Shoulder pShoulder,
      Wrist pWrist,
      Travelator pTravelator) {

    drivetrain = pDrivetrain;
    dislocator = pDislocator;
    elbow = pElbow;
    shoulder = pShoulder;
    wrist = pWrist;
    travelator = pTravelator;

    blueCommandSequence =
        Commands.sequence(
            new DropLowPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2.5),
            new GrabOpen(wrist).withTimeout(1),
            Commands.parallel(
                    // new DriveAuto(pDrivetrain, -.3).withTimeout(.2),
                    new CompressPosition(pDislocator, pElbow, pShoulder, pTravelator))
                .withTimeout(1.5),
            new DriveAuto(pDrivetrain, -.4).withTimeout(2.83),
            new DriveAuto(pDrivetrain, 0).withTimeout(.9),
            new DriveAuto(pDrivetrain, .3).withTimeout(1.5),
            new DrivetrainBalancing(drivetrain, 0, 0).withTimeout(8));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blueCommandSequence.schedule();
  }
}
