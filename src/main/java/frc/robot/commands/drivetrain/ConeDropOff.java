// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.LimeLight;

public class ConeDropOff extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private LimeLight m_Vision;
  private double X;
  private XboxController Opcontroller;

  /** Creates a new DriveTele. */
  public ConeDropOff(XboxController pController, Drivetrain subsystem, LimeLight pLimeLight) {
    m_Drivetrain = subsystem;
    m_Vision = pLimeLight;
    Opcontroller = pController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Vision.getTargetX() != 0) {
      if (m_Vision.getTargetY() > 5) {

        if ((m_Vision.getTargetX() + 8) < -2) {
          X = ((m_Vision.getTargetX() + 8) * .014) - .15;
        } else if ((m_Vision.getTargetX() + 8) > 2) {
          X = ((m_Vision.getTargetX() + 8) * .014) + .15;
        } else {
          X = 0;
        }

        m_Drivetrain.mecanumDrive(X, Opcontroller.getRawAxis(1), Opcontroller.getRawAxis(4));
      } else {

        if (m_Vision.getTargetX() < -2) {
          X = (m_Vision.getTargetX() * .014) - .15;
        } else if (m_Vision.getTargetX() > 2) {
          X = (m_Vision.getTargetX() * .014) + .15;
        } else {
          X = 0;
        }

        m_Drivetrain.mecanumDrive(X, Opcontroller.getRawAxis(1), Opcontroller.getRawAxis(4));
      }
    } else {
      m_Drivetrain.mecanumDrive(
          Opcontroller.getRawAxis(0), Opcontroller.getRawAxis(1), Opcontroller.getRawAxis(4));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.mecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
