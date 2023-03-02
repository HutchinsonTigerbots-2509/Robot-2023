// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.PhotonVision;

public class CubeDropOff extends CommandBase {
  private final Drivetrain Dt;
  private PhotonVision photonvision;
  private double X;
  private XboxController OpController;

  /** Creates a new DriveTele. */
  public CubeDropOff(XboxController pController, Drivetrain pDt, PhotonVision pPhotonVision) {
    Dt = pDt;
    photonvision = pPhotonVision;
    OpController = pController;
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
    if (photonvision.fetchTargetX() < -4) {
      X = (photonvision.fetchTargetX() * .014) - .15;
    } else if (photonvision.fetchTargetX() > 4) {
      X = (photonvision.fetchTargetX() * .014) + .15;
    } else {
      X = 0;
    }

    SmartDashboard.putNumber("Photon X", photonvision.fetchTargetX());

    //Dt.TeleMecDrive(stick.getY(), X, stick.getZ());
    Dt.mecanumDrive(X, OpController.getRawAxis(1), OpController.getRawAxis(4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Dt.TeleMecDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
