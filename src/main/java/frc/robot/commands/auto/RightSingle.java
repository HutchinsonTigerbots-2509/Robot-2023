// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Rightsingle extends SequentialCommandGroup {
  /** Creates a new Rightsingle. */
  public Rightsingle() {
    Alliance color = DriverStation.getAlliance();

    if (color == Alliance.Blue) {
    } else {
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
} // Drop sequence 3

// Move grabber to oposite side

// Drive forward and strafe left

// Grab cube

// Move grabber to drop position

// Drive back

// Drop cube
