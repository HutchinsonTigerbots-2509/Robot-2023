// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPosition;
import frc.robot.subsystems.Drivetrain;
import frc
Drivetrain sDrivetrain; 

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class P1Double extends SequentialCommandGroup {
  /** Creates a new LeftSingle. */
  public P1Double(Drivetrain sDrivetrain, Pose2d targetPose ) {
    Alliance color = DriverStation.getAlliance();

    if (color == Alliance.Blue) {

      //1.944496403743151 x
      //0.477247225100959 y start one blue
      new DriveToPosition(sDrivetrain, new Pose2d(1.944496403743151,0.477247225100959,new Rotation2d()));

      //6.202532950021199 x
      //0.815038327521982 y
      new DriveToPosition(sDrivetrain, new Pose2d(6.202532950021199,0.815038327521982,new Rotation2d()));

      //1.16298497890432 x
      //2.030060169629243 y
      new DriveToPosition(sDrivetrain, new Pose2d(1.16298497890432,2.030060169629243,new Rotation2d()));

    } else {
     
      //14.634718153442892 x
//0.382807439027479 y start one red
new DriveToPosition(sDrivetrain, new Pose2d(14.634718153442892,0.382807439027479,new Rotation2d()));
//10.468843760267244 x
//0.886324254209018 y 
new DriveToPosition(sDrivetrain, new Pose2d(10.468843760267244,0.886324254209018,new Rotation2d()));
//14.53695576595974 x
//1.091239527336539 y
new DriveToPosition(sDrivetrain, new Pose2d(14.53695576595974,1.091239527336539,new Rotation2d()));


    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
} // Drop sequence 3

// Move grabber across

// Move forward and strafe right

// Grab block
