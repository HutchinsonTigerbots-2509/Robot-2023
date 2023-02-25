// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ctrlConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.subsystems.Vision.PhotonVision;

public class DriveVision extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private Joystick m_joystick;
  private LimeLight m_Vision;
  private double turn;

  /** Creates a new DriveTele. */
  public DriveVision(Joystick stick, Drivetrain subsystem, LimeLight pVision) {
    m_Drivetrain = subsystem;
    m_joystick = stick;
    m_Vision = pVision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Do nothing
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if(m_Vision.getTargetX() < 145) {turn = ((m_Vision.getTargetX() - 30) * .01) - .08;}
    else if (m_Vision.getTargetX() > 155) {turn = ((m_Vision.getTargetX() - 30) * .010) + .08;}
    else {turn = 0;}

    SmartDashboard.putNumber("April Tag X", m_Vision.getTargetX());

    m_Drivetrain.TeleMecDrive(

      -m_joystick.getRawAxis(ctrlConstants.kXboxLeftJoystickY),
      m_Drivetrain.GetStrafeValue(m_joystick),
      turn);
      m_joystick.setRumble(RumbleType.kLeftRumble, 1);
      m_joystick.setRumble(RumbleType.kRightRumble, 1);


      
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.TeleMecDrive(0, 0, 0);
    m_joystick.setRumble(RumbleType.kLeftRumble, 0);
    m_joystick.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
