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
public class DriveAutoGyro extends PIDCommand {

  static final double kP = 0.05;
  static final double kI = 0.005;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /** Creates a new AutoDriveGyro. */
  public DriveAutoGyro(Drivetrain Dt, double y, double x, double zPos) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        Dt::getAngle,
        // This should return the setpoint (can also be a constant)
        zPos,
        // This uses the output
        output -> {
          // Use the output here
          Dt.mecanumDrive(x, y, output);;
        });
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Dt);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(2);
    this.getController().setSetpoint(zPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
