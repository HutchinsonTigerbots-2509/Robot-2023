// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ***** Joystick ID's ***** //
  public static final int kOpStickID = 0;
  public static final int kCoopStickID = 1;

  public static final class opConstants {

    // ***** Drivetrain Constants ***** //
    public static final int kFrontLeftID = 14;
    public static final int kFrontRightID = 1;
    public static final int kRearLeftID = 15;
    public static final int kRearRightID = 0;
    public static final double kMaxSpeed = 0.8;
    public static final double kMaxAngularSpeed = 0.8;
    public static final double kHighSpeedStrafe = 1;
    public static final double kLowSpeedStrafe = .6;
    public static final double kSkewRateLimit = 3;
    public static final double kGearRatio = 20.0;
    public static final int kFalconUnitsPerRotation = 2048;
    public static final double kWheelDiameter = 10.0; // This is in centimeters.

    // ***** Intake Constants ***** //
    public static final int kIntakeMotorID_0 = 11; // Motor ID for the front intake - S. COllins
    public static final int kIntakeMotorID_1 =
        12; // Motor ID for the running intake to shooter - S.Collins
    public static final double kIntakeSpeed = .5;

    // ***** Shooter Constants ***** //
    public static final int kShooterMotorID = 13;
    public static final int kFlapperMotorID = 8;
    public static final int kShooterDistance = 5;
    public static final double kFlapGoalPosition = 24;
    public static final double kShootingSpeed =
        .6; // Changed to .6 3/19/22  // <------------ TO CHANGE SHOOTING SPEED

    // ***** Conveyor Constants ***** //
    public static final int kConveyorMotorID = 12;
    public static final double kMaxConveyorSpeed = .75;
    public static final int kLightSensor = 1;

    // ***** Climb Constants ***** //
    public static final int kClimberMotorID = 2;

    // ***** Arm Constants ***** //
    public static final int kArmMotor1ID = 0;
  }

  public static final class camConstants {

    // #region ***** Vision Constants ***** //

    // Network
    public static final String kLimelightIP = "10.25.9.11"; // IP Address of Camera
    public static final String kLimelightNetworkID = "limelight"; // Name of Camera on Network

    // Settings
    public static final int kLimelightLED =
        0; // Sets LED. 0 = Set by Pipline, 1 = Force off, 2 = Force blink, 3 = Force on
    public static final int kLimelightMode =
        0; // Sets camera mode. 0 = Vision processor, 1 = Driver Camera
    public static final int kLimelightStream =
        0; // Sets streaming mode. 0 = Side-by-Side, 1 = PiP main, 2 = PiP secondary
    public static final int kLimelightStartingPipeline = 1; // The default pipeline to stream

    // Table IDs (for getting values from the Network Table)
    public static final String kLimelightLatencyID = "tl"; // Pipeline latency in milliseconds
    public static final String kLimelightTargetID =
        "tv"; // Whether or not a valid target is found (0 or 1)
    public static final String kLimelightTargetXID =
        "tx"; // Horizontal offset from crosshair to target (+/- 27 degrees)
    public static final String kLimelightTargetYID =
        "ty"; // Vertical offset from crosshair to target (+/- 20.5 degrees)
    public static final String kLimelightTargetAreaID = "ta"; // Target area (0-100 % of image)
    public static final String kLimelightTargetSkewID =
        "ts"; // Target skew/rotation (-90 to 0 degrees)
    public static final String kLimelightTargetVertID =
        "tvert"; // Vertical sidelength of bounding box (0-320 pixels)
    public static final String kLimelightTargetHorID =
        "thor"; // Horizontal sidelength of bounding box (0-320 pixels)

    // Camera Variables
    public static final double kCameraHeight = 4;
    // public static double kCameraAngle = -28.23744554;
    public static final double kCameraAngle = -31.47286489;
    public static final double kTargetHeight = 31.5;
    public static final double kpAim = -0.02;
    public static final double kpDistance = -0.05;
    public static final double kmin_aim_command = -0.5;
    public static final double kdistance_command = -0.5;
    public static final double kTargetDistanceFromTarget = 24;
    // #endregion

    // Constants for April Tags using PhotonVision below

    private static final double kFieldLength = Units.inchesToMeters(651.2);
    private static final double kFieldWidth = Units.inchesToMeters(315.75);

    public static final double camFOV =
        0; // needs to be replaced with what's in http://photonvision:5800 settings
    public static final int camResWidth = 0; // "
    public static final int camResHeight = 0; // "
    public static final double minClosestTargetDistance =
        0; // Set this to what we plan on using, probably should play around with a good distance.
    // Goes by meters

    public static final AprilTagFieldLayout tagPlayground =
        new AprilTagFieldLayout(null, kFieldLength, kFieldWidth);

    public static Transform3d robotToCamera =
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }

  public static final class ctrlConstants {

    // #region ***** Xbox Controller Constants ***** //
    public static final int kXboxLeftJoystickX = 0;
    public static final int kXboxLeftJoystickY = 1;

    public static final int kXboxRightJoystickX = 4;
    public static final int kXboxRightJoystickY = 5;

    public static final int kXboxLeftJoystickButton = 9;
    public static final int kXboxRightJoystickButton = 10;

    public static final int kXboxLeftTrigger = 2;
    public static final int kXboxRightTrigger = 3;

    public static final int kXboxLeftBumper = 5;
    public static final int kXboxRightBumper = 6;

    public static final int kXboxButtonA = 1;
    public static final int kXboxButtonB = 2;
    public static final int kXboxButtonX = 3;
    public static final int kXboxButtonY = 4;

    // TODO Assign ID's for the Xbox DPad

    public static final int kXboxButtonBack = 7;
    public static final int kXboxButtonStart = 8;
    // #endregion

    // #region ***** Joystick Buttons ***** //
    public static final int kJoystickX = 0;
    public static final int kJoystickY = 1;
    public static final int kJoystickZ = 2;

    public static final int kJoystickSlider = 3;

    public static final int kJoystickButton1 = 1;
    public static final int kJoystickButton2 = 2;
    public static final int kJoystickButton3 = 3;
    public static final int kJoystickButton4 = 4;
    public static final int kJoystickButton5 = 5;
    public static final int kJoystickButton6 = 6;
    public static final int kJoystickButton7 = 7;
    public static final int kJoystickButton8 = 8;
    public static final int kJoystickButton9 = 9;
    public static final int kJoystickButton10 = 10;
    public static final int kJoystickButton11 = 11;
    public static final int kJoystickButton12 = 12;
  }
}
