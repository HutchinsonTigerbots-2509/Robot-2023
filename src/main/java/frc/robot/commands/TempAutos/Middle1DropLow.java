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
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.commands.drivetrain.DriveAuto;
import frc.robot.commands.drivetrain.DrivetrainBalancing;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Grabber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Middle1DropLow extends InstantCommand {
  private Command blueCommandSequence;
  private Command redCommandSequence;

  Drivetrain drivetrain;
  Dislocator dislocator;
  Elbow elbow;
  Shoulder shoulder;
  Grabber grabber;
  Travelator travelator;

  /** Creates a new LeftSingleCharger. */
  public Middle1DropLow(
      Drivetrain pDrivetrain,
      Dislocator pDislocator,
      Elbow pElbow,
      Shoulder pShoulder,
      Grabber pGrabber,
      Travelator pTravelator) {

    drivetrain = pDrivetrain;
    dislocator = pDislocator;
    elbow = pElbow;
    shoulder = pShoulder;
    grabber = pGrabber;
    travelator = pTravelator;

    blueCommandSequence =
        Commands.sequence(
            Commands.parallel(
                    new DislocatorMoveToPosition(0, dislocator),
                    new ShoulderMoveToPosition(-230, shoulder),
                    new ElbowMoveToPosition(30, elbow)).withTimeout(2),
            new TravelatorMoveToPosition(opConstants.kTravelatorFront - 6, travelator)
                .withTimeout(2),
            new GrabOpen(grabber).withTimeout(1),
            Commands.parallel(
                new DriveAuto(pDrivetrain, -.3),
                new ShoulderMoveToPosition(-330, shoulder),
                new TravelatorMoveToPosition(opConstants.kTravelatorMiddle, travelator),
                new DislocatorMoveToPosition(0, dislocator),
                new ElbowMoveToPosition(154, elbow)).withTimeout(2.5),
            new DrivetrainBalancing(drivetrain, 0, 0).withTimeout(8));

    redCommandSequence =
        Commands.sequence(
            Commands.parallel(
                    new DislocatorMoveToPosition(0, dislocator),
                    new ShoulderMoveToPosition(-230, shoulder),
                    new ElbowMoveToPosition(30, elbow)).withTimeout(2),
            new TravelatorMoveToPosition(opConstants.kTravelatorFront - 6, travelator)
                .withTimeout(2),
            new GrabOpen(grabber).withTimeout(1),
            Commands.parallel(
                new DriveAuto(pDrivetrain, -.3),
                new ShoulderMoveToPosition(-330, shoulder),
                new TravelatorMoveToPosition(opConstants.kTravelatorMiddle, travelator),
                new DislocatorMoveToPosition(0, dislocator),
                new ElbowMoveToPosition(154, elbow)).withTimeout(2.5),
            new DrivetrainBalancing(drivetrain, 0, 0).withTimeout(8));

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
