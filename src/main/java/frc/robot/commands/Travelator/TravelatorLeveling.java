// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Travelator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelatorLeveling extends PIDCommand {
  static final double kP = 0.03;
  static final double kI = 0.003;
  static final double kD = 0.00;
  static final double kF = 0.00;

  /** Creates a new TravelatorLeveling. */
  public TravelatorLeveling(Travelator tv, Drivetrain Dt) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        Dt::getRoll,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          tv.Move(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(tv);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(0.25);
    this.getController().setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
