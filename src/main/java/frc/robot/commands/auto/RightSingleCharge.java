// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSingleCharge extends SequentialCommandGroup {
  Alliance color;

  /** Creates a new RightSingleCharge. */
  public RightSingleCharge() {
    color = DriverStation.getAlliance();

    if (color == Alliance.Blue) {// Right single charger

      // Drop sequence 3 
      
      // Move grabber to opposite side
      
      // Drive forward and strafe left
      
      // Grab cube
      
      // Strafe left
      
      // Drive back
      
      // Deploy compressor
      // Blue Alliance Auto
    } else {
      // Red Alliance Auto
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
