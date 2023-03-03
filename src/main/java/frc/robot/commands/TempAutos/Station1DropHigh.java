// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TempAutos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.opConstants;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Grabber.GrabOpen;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.Arm.Wrist.WristMoveToPosition;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
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
public class Station1DropHigh extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Wrist wrist;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public Station1DropHigh(
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

    Commands.parallel(
      new TravelatorMoveToPosition(opConstants.kTravelatorFront, travelator),
      new DislocatorMoveToPosition(-22, dislocator),
      new ShoulderMoveToPosition(-199.5, shoulder),
      new ElbowMoveToPosition(27.5, elbow),
      new WristMoveToPosition(0, wrist)).withTimeout(2),

    new GrabOpen(wrist).withTimeout(1),

    Commands.parallel(
        new DriveAuto(pDrivetrain, -.4).withTimeout(1.5),
        // new ElbowMoveToPosition(0, pElbow),
        new ShoulderMoveToPosition(300, shoulder)));

    redCommandSequence =
    Commands.sequence(

    Commands.parallel(
      new TravelatorMoveToPosition(opConstants.kTravelatorFront, travelator),
      new DislocatorMoveToPosition(-22, dislocator),
      new ShoulderMoveToPosition(-199.5, shoulder),
      new ElbowMoveToPosition(27.5, elbow),
      new WristMoveToPosition(0, wrist)).withTimeout(2),

    new GrabOpen(wrist).withTimeout(1),

    Commands.parallel(
        new DriveAuto(pDrivetrain, -.4).withTimeout(1.5),
        // new ElbowMoveToPosition(0, pElbow),
        new ShoulderMoveToPosition(300, shoulder)));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      blueCommandSequence.schedule();
    } else {
      redCommandSequence.schedule();
    }
  }
}
