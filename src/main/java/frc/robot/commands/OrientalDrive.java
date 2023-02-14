// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OrientalDrive extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private XboxController m_joystick;

  /** Creates a new DriveTele. */
  public OrientalDrive(XboxController stick, Drivetrain subsystem) {
    m_Drivetrain = subsystem;
    m_joystick = stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Do nothing
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.OrientDrive(-m_joystick.getLeftY(), m_joystick.getLeftX(), m_joystick.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
