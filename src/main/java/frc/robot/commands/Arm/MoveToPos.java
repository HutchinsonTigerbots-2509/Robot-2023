// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.subsystems.Travelator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToPos extends SequentialCommandGroup {
  /** Creates a new MoveToPos. */

  Shoulder sShoulder;
  Travelator sTravelator;
  Dislocator sDislocator;
  Elbow sElbow;
  
  public MoveToPos(Shoulder pShoulder, Dislocator pDislocator, Elbow pElbow, Travelator pTravelator, 
                    double ShoulderPos, double DislocatorPos, double ElbowPos, double TravelatorPos, Boolean ElbowOn, double wait) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    sShoulder = pShoulder;
    sTravelator = pTravelator;
    sDislocator = pDislocator;
    sElbow = pElbow;
    
    if (ElbowOn == false) {
      addCommands(
        new ShoulderMoveToPosition(ShoulderPos, sShoulder).withTimeout(wait),
        Commands.parallel(
          new ShoulderMoveToPosition(ShoulderPos, sShoulder),
          new TravelatorMoveToPosition(TravelatorPos, sTravelator),
          new DislocatorMoveToPosition(DislocatorPos, sDislocator),
          new ElbowMoveToPosition(ElbowPos, pElbow)
        )
      );
    }
    else {
      addCommands(
      Commands.parallel(
        new ShoulderMoveToPosition(ShoulderPos, sShoulder),
        new ElbowMoveToPosition(ElbowPos, sElbow)).withTimeout(wait),
      Commands.parallel(
        new ShoulderMoveToPosition(ShoulderPos, sShoulder),
        new TravelatorMoveToPosition(TravelatorPos, sTravelator),
        new DislocatorMoveToPosition(DislocatorPos, sDislocator),
        new ElbowMoveToPosition(ElbowPos, pElbow)
      )
    );
    }
  }
}
