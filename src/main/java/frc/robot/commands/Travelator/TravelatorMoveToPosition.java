// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Travelator;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.opConstants;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelatorMoveToPosition extends PIDCommand {
  static final double kP = 0.045;
  static final double kI = 0.008;
  static final double kD = 0.00;
  static final double kF = 0.00;
  Joystick buttonBoardRight = new Joystick(Constants.kButtonBoardRightID);

  /** Creates a new TravelatorMoveToPosition. */
  public TravelatorMoveToPosition(double TargetDistance, Travelator tv) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        tv::getTravelatorPos,
        // This should return the setpoint (can also be a constant)
        TargetDistance,
        // This uses the output
        output -> {
          // Use the output here4
          tv.Move(output);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tv);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
