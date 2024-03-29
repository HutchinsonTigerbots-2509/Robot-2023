// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arms.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristMoveToPosition extends PIDCommand {
  static final double kP = 0.4;
  static final double kI = 0.0;
  static final double kD = 0.0;
  /** Creates a new WristMoveToPosition. */
  public WristMoveToPosition(double PreferredAngle, Wrist wrist) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        wrist::getWristPose,
        // This should return the setpoint (can also be a constant)
        PreferredAngle,
        // This uses the output
        output -> {
          // Use the output here
          wrist.WristMove(-output);
        });
    // Use addRequirements() here to declare subsystem depePndencies.
    addRequirements(wrist);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(4.0);
    this.getController().setSetpoint(PreferredAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atSetpoint();
  }
}
