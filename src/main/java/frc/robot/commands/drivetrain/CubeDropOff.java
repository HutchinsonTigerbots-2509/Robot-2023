// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.PhotonVision;

public class CubeDropOff extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private PhotonVision m_Vision;
  private double X;
  private Joystick stick;

  /** Creates a new DriveTele. */
  public CubeDropOff(Joystick pstick, Drivetrain subsystem, PhotonVision pPhotonVision) {
    m_Drivetrain = subsystem;
    m_Vision = pPhotonVision;
    stick = pstick;
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
    if(m_Vision.fetchTargetX() < -4) {X = (m_Vision.fetchTargetX() * .014) - .15;}
    else if (m_Vision.fetchTargetX() > 4) {X = (m_Vision.fetchTargetX() * .014) + .15;}
    else {X = 0;}

    SmartDashboard.putNumber("x", m_Vision.fetchTargetX());
    m_Drivetrain.AutoMecDrive(
      stick.getY(),
      X,
      stick.getZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.AutoMecDrive(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}