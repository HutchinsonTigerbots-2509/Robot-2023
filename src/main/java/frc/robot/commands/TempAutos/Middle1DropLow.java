// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TempAutos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.drivetrain.DriveAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;

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
  Wrist wrist;

  /** Creates a new LeftSingleCharger. */
  public Middle1DropLow(
    Drivetrain pDrivetrain, Dislocator pDislocator, Elbow pElbow, Shoulder pShoulder, Wrist pWrist) {

    drivetrain = pDrivetrain;
    dislocator = pDislocator;
    elbow = pElbow;
    shoulder = pShoulder;
    wrist = pWrist;
  
    blueCommandSequence =
        Commands.sequence(
          new DropLowPosition(dislocator, elbow, shoulder, wrist),
          //new Wrist.cmdGrabOpen(),
          new DriveAuto(pDrivetrain, -.6).withTimeout(1)
        );
    redCommandSequence =
        Commands.sequence(
          new DropLowPosition(dislocator, elbow, shoulder, wrist),
          new DriveAuto(pDrivetrain, -.6).withTimeout(1)
        );
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