// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoulder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arms.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShoulderMoveToPosition extends PIDCommand {
  static final double kP = 0.4;
  static final double kI = 0.00;
  static final double kD = 0.00;
  /** Creates a new Shoulder. */
  public ShoulderMoveToPosition(double PreferredAngle, Shoulder shoulder) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        shoulder::getShoulderPos,
        // This should return the setpoint (can also be a constant)
        PreferredAngle,
        // This uses the output
        output -> {
          // Use the output here
          shoulder.ShoulderMove(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
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
