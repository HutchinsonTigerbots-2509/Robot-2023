// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PermAutos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.Grabber.GrabClose;
import frc.robot.commands.Arm.Grabber.GrabOpen;
import frc.robot.commands.PresetPoses.DropGroundPosition;
import frc.robot.commands.PresetPoses.DropHighPosition;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.PresetPoses.GrabBackPosition;
import frc.robot.commands.PresetPoses.GrabDropPosition;
import frc.robot.commands.PresetPoses.TuckPosition;
import frc.robot.commands.drivetrain.DriveAuto;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeDropLowGrab extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Wrist wrist;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public ConeDropLowGrab(
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
            new DropLowPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(3),
            new GrabOpen(wrist).withTimeout(1),
            new DriveAuto(drivetrain, -.3).withTimeout(.5),
            Commands.parallel(
                    new DriveAuto(drivetrain, -.4).withTimeout(1.80),
                    new GrabBackPosition(pDislocator, pElbow, pShoulder, pTravelator))
                .withTimeout(3),
            new DriveAuto(drivetrain, -.25).withTimeout(.75),
            new GrabClose(wrist).withTimeout(.5),
            Commands.parallel(
                    new DriveAuto(drivetrain, .4).withTimeout(2.65),
                    new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator)).withTimeout(2.6),
            new DropGroundPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2.6),
            new GrabOpen(pWrist));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blueCommandSequence.schedule();
  }
}
