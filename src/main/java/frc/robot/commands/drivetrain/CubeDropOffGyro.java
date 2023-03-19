// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.PhotonVision;

public class CubeDropOffGyro extends CommandBase {
  private final Drivetrain Dt;
  private PhotonVision photonvision;
  private double X;
  private XboxController OpController;
  private PIDController rotController;

  static final double kP = 0.2;
  static final double kI = 0.001;
  static final double kD = 0.00;
  static final double minSpeed = 0.12;

  /** Creates a new DriveTele. */
  public CubeDropOffGyro(XboxController pController, Drivetrain pDt, PhotonVision pPhotonVision) {
    Dt = pDt;
    photonvision = pPhotonVision;
    OpController = pController;
    this.rotController = new PIDController(kP, kI, kD);
    this.rotController.setTolerance(2);
    this.rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare pDt dependencies.
    addRequirements(Dt);
    addRequirements(photonvision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotSpeed;
    rotSpeed = rotController.calculate(Dt.getAngle());

    if (photonvision.fetchTargetX() != 0) {
      if ((photonvision.fetchTargetX() - 380) < -25) {
        X = ((photonvision.fetchTargetX() - 380) * .00008) - .15;
      } else if ((photonvision.fetchTargetX() - 380) > 25) {
        X = ((photonvision.fetchTargetX() - 380) * .00008) + .15;
      } else {
        X = 0;
      }

      SmartDashboard.putNumber("Photon X", photonvision.fetchTargetX());

      // Dt.TeleMecDrive(stick.getY(), X, stick.getZ());
      Dt.mecanumDrive(X, OpController.getRawAxis(1), rotSpeed);
    } else {
      Dt.mecanumDrive(
          OpController.getRawAxis(0), OpController.getRawAxis(1), OpController.getRawAxis(4));
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
