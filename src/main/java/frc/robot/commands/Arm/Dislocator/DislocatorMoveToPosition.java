// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Dislocator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arms.Dislocator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DislocatorMoveToPosition extends PIDCommand {
  static final double kP = 0.15;
  static final double kI = 0.05;
  static final double kD = 0.0;
  /** Creates a new DislocatorMoveToPosition. */
  public DislocatorMoveToPosition(double PreferredPosition, Dislocator dislocator) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        dislocator::getDislocatorPos,
        // This should return the setpoint (can also be a constant)
        PreferredPosition,
        // This uses the output
        output -> {
          // Use the output here
          dislocator.DislocatorMove(output);
        });
    // Use addRequirements() here to declare subsystem depePndencies.
    addRequirements(dislocator);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(0.4);
    this.getController().setSetpoint(PreferredPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
