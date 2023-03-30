// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TempAutos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.Grabber.GrabClose;
import frc.robot.commands.Arm.Grabber.GrabOpen;
import frc.robot.commands.PresetPoses.DropHighPosition;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.PresetPoses.GrabBackPosition;
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
public class Station2Drop extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Wrist wrist;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public Station2Drop(
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
            new DropHighPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2),
            new GrabOpen(wrist).withTimeout(1),
            new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2),
            new DriveAuto(drivetrain, -.3).withTimeout(3.2),
            new DriveAuto(pDrivetrain, 0, -.3, 0).withTimeout(.5),
            new GrabBackPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(3),
            new DriveAuto(pDrivetrain, -.3).withTimeout(.3),
            new GrabClose(wrist).withTimeout(.5),
            new DriveAuto(pDrivetrain, 0, .3, 0).withTimeout(.5).withTimeout(3),
            new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator),
            new DriveAuto(drivetrain, .3).withTimeout(3.5),
            new DropLowPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2),
            new GrabOpen(wrist).withTimeout(1));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blueCommandSequence.schedule();
  }
}
