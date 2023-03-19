// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.LimeLight;

public class ConeDropOffGyro extends CommandBase {
  private final Drivetrain Dt;
  private LimeLight m_Vision;
  private double X;
  private XboxController Opcontroller;
  private PIDController rotController;

  static final double kP = 0.2;
  static final double kI = 0.001;
  static final double kD = 0.00;
  static final double minSpeed = 0.12;


  /** Creates a new DriveTele. */
  public ConeDropOffGyro(XboxController pController, Drivetrain subsystem, LimeLight pLimeLight) {
    Dt = subsystem;
    m_Vision = pLimeLight;
    Opcontroller = pController;
    this.rotController = new PIDController(kP, kI, kD);
    this.rotController.setTolerance(2);
    this.rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Dt);
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotSpeed;
    rotSpeed = rotController.calculate(Dt.getAngle());

    if (m_Vision.getTargetX() != 0) {
      if (m_Vision.getTargetY() > 5) {

        if ((m_Vision.getTargetX() + 8) < -2) {
          X = ((m_Vision.getTargetX() + 8) * .014) - .15;
        } else if ((m_Vision.getTargetX() + 8) > 2) {
          X = ((m_Vision.getTargetX() + 8) * .014) + .15;
        } else {
          X = 0;
        }

        Dt.mecanumDrive(X, Opcontroller.getRawAxis(1), rotSpeed);
      } else {

        if (m_Vision.getTargetX() < -2) {
          X = (m_Vision.getTargetX() * .014) - .15;
        } else if (m_Vision.getTargetX() > 2) {
          X = (m_Vision.getTargetX() * .014) + .15;
        } else {
          X = 0;
        }

        Dt.mecanumDrive(X, Opcontroller.getRawAxis(1), rotSpeed);
      }
    } else {
      Dt.mecanumDrive(
          Opcontroller.getRawAxis(0), Opcontroller.getRawAxis(1), Opcontroller.getRawAxis(4));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dt.mecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
