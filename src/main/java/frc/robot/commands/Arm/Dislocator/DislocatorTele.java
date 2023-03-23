// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Dislocator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms.Dislocator;

public class DislocatorTele extends CommandBase {

  Joystick buttonBoard;
  Dislocator dislocator;

  /** Creates a new TravelatorTele. */
  public DislocatorTele(Dislocator pDislocator, Joystick pButtonBoard) {

    buttonBoard = pButtonBoard;
    dislocator = pDislocator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dislocator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dislocator.MoveTele(buttonBoard);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
