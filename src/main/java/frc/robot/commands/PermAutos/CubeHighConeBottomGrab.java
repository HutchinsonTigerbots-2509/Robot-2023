// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PermAutos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.Grabber.GrabClose;
import frc.robot.commands.Arm.Grabber.GrabOpen;
import frc.robot.commands.PresetPoses.DropGroundPosition;
import frc.robot.commands.PresetPoses.DropHighPosition;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.PresetPoses.GrabBackPosition;
import frc.robot.commands.PresetPoses.GrabPosition;
import frc.robot.commands.PresetPoses.TuckPosition;
import frc.robot.commands.drivetrain.DriveAutoGyro;
import frc.robot.commands.drivetrain.DriveAutoGyro;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeHighConeBottomGrab extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Wrist wrist;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public CubeHighConeBottomGrab(
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
    new DropHighPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2), // with droplow the timeout is 3
    new GrabOpen(wrist).withTimeout(1),
    Commands.parallel(
      new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator),
      new DriveAutoGyro(drivetrain, -.3, 0)).withTimeout(.5),
    Commands.parallel(
      new GrabBackPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(3),
      new DriveAutoGyro(drivetrain, -.6, -2).withTimeout(0.6).andThen(
      new DriveAutoGyro(drivetrain, -.5, -2).withTimeout(0.4).andThen(
      new DriveAutoGyro(drivetrain, -.4, -2).withTimeout(0.4).andThen(
      new DriveAutoGyro(drivetrain, -.3, -2).withTimeout(0.5).andThen(
      new DriveAutoGyro(drivetrain, -.2, 0).withTimeout(0.5)))))),
    new GrabClose(wrist).withTimeout(.5),
    Commands.parallel(
      new DriveAutoGyro(drivetrain, .4, 0).withTimeout(.3).andThen(
      new DriveAutoGyro(drivetrain, .8, 0).withTimeout(1).andThen(
      new DriveAutoGyro(drivetrain, .4, 0).withTimeout(.6))),
      new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator).andThen(
      new DropGroundPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(1.25).andThen(
      new GrabOpen(pWrist).withTimeout(.3)))),
    Commands.parallel(
      new DriveAutoGyro(drivetrain, -.8, 0).withTimeout(1.45),
      new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator)),
    Commands.parallel(
      new GrabPosition(pDislocator, pElbow, pShoulder, pTravelator),
      new DriveAutoGyro(drivetrain, 0, -90)).withTimeout(1.5));

    
      redCommandSequence =
      Commands.sequence(
      new DropHighPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(2), // with droplow the timeout is 3
      new GrabOpen(wrist).withTimeout(1),
      Commands.parallel(
        new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator),
        new DriveAutoGyro(drivetrain, -.3, 0)).withTimeout(.5),
      Commands.parallel(
        new GrabBackPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(3),
        new DriveAutoGyro(drivetrain, -.6, 2).withTimeout(0.6).andThen(
        new DriveAutoGyro(drivetrain, -.5, 2).withTimeout(0.4).andThen(
        new DriveAutoGyro(drivetrain, -.4, 2).withTimeout(0.4).andThen(
        new DriveAutoGyro(drivetrain, -.3, 2).withTimeout(0.5).andThen(
        new DriveAutoGyro(drivetrain, -.2, 0).withTimeout(0.5)))))),
      new GrabClose(wrist).withTimeout(.5),
      Commands.parallel(
        new DriveAutoGyro(drivetrain, .4, 0).withTimeout(.3).andThen(
        new DriveAutoGyro(drivetrain, .8, 0).withTimeout(1).andThen(
        new DriveAutoGyro(drivetrain, .4, 0).withTimeout(.6))),
        new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator).andThen(
        new DropGroundPosition(pDislocator, pElbow, pShoulder, pTravelator).withTimeout(1.25).andThen(
        new GrabOpen(pWrist).withTimeout(.3)))),
      Commands.parallel(
        new DriveAutoGyro(drivetrain, -.8, 0).withTimeout(1.45),
        new TuckPosition(pDislocator, pElbow, pShoulder, pTravelator)),
      Commands.parallel(
        new GrabPosition(pDislocator, pElbow, pShoulder, pTravelator),
        new DriveAutoGyro(drivetrain, 0, 90)).withTimeout(1.5));


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance color = DriverStation.getAlliance();

    if (color == Alliance.Blue) {
      blueCommandSequence.schedule();
    } else {
      redCommandSequence.schedule();
    }
  }
}
