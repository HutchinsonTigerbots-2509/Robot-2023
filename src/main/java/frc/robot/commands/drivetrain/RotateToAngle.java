// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAngle extends PIDCommand {
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(double TargetAngle, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drive::getAngle,
        // This should retuPrn the setpoint (can also be a constant)
        TargetAngle,
        // This uses the output
        output -> {
          drive.arcadeDrive(0, output);
        },
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180.0, 180.0);
    getController().setTolerance(2.0f);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(getController().atSetpoint());
    return getController().atSetpoint();
  }
}
