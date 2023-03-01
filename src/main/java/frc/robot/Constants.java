// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  // Joystick ID's
  public static final int kOpStickID = 0;
  public static final int kCoopStickID = 1;

  public static final class opConstants {

    // Drivetrain Constants
    public static final int kFrontLeftID = 0;
    public static final int kFrontRightID = 15;
    public static final int kRearLeftID = 1;
    public static final int kRearRightID = 14;
    public static final double kMaxSpeed = 0.8;
    public static final double kMaxAngularSpeed = 0.5;
    public static final double kHighSpeedStrafe = 0.8;
    public static final double kLowSpeedStrafe = .5;
    public static final double kSkewRateLimit = 0.9;
    public static final double kGearRatio = 9.0;
    public static final int kFalconUnitsPerRotation = 2048;
    public static final double kWheelDiameter = 15.5; // This is in centimeters.
    public static final int kParkingBrakeExtend = 2;
    public static final int kParkingBrakeRetract = 3;

    // Travelator Constants
    public static final int kTravelatorID = 2;
    public static final double kTravelatorSpeed = .4;
    public static final int kBackRightLimitSwitchID = 0;
    public static final int kBackLeftLimitSwitchID = 1;
    public static final int kFrontRightLimitSwitchID = 2;
    public static final int kFrontLeftLimitSwitchID = 3;
    public static final double kTravelatorGearRatio = 1.9411764 * 4;
    public static final double kTravelatorMax = 113800;
    public static final double kTravelatorMin = 0;
    public static final double kTravelatorBack = 0;
    public static final double kTravelatorMiddle = 10;
    public static final double kTravelatorFront = 19;

    // ***** Shoulder Constants ***** //
    public static final int kShoulderID = 4;
    public static final double kShoulderGearRatio = 192;
    public static final double kShoulderSpeed = .8;
    public static final int kArmCounterID = 0;

    // ** Elbow Constants */
    public static final int kArmElbowID = 11;
    public static final double kElbowSpeed = .8;
    public static final int kElbowEncoder1ID = 4;
    public static final int kElbowEncoder2ID = 5;

    // ** Wrist Constants */
    public static final int kArmWristID = 10;
    public static final double kWristSpeed = .8;
    public static final int kWristEncoder1ID = 6;
    public static final int kWristEncoder2ID = 7;

    // ***** Dislocator Constants ***** //
    public static final int kDislocatorID = 3;
    public static final double kDislocatorSpeed = .6;
    public static final double kDislocatorGearRatio = 20; // 20

    public static final double kMaxArmSpeed = .45;
    public static final int kWristFullRotation = 90;

    // * Solenoids */
    public static final int kGrabberP1 = 0;
    public static final int kGrabberP2 = 1;
    public static final int kParkingBrakeP1 = 2;
    public static final int kParkingBrakeP2 = 3;
  }

  public static final class camConstants {

    // Network
    public static final String kPhotonCameraID = "OV5746"; // Name of Camera on Network
    public static final String kLimelightCameraID = "limelight"; // Limelight Name
    public static final String kLimelightIP = "10.25.9.11"; // IP Address of Camera
    

    // Settings
    // Sets LED. 0 = Set by Pipline, 1 = Force off, 2 = Force blink, 3 = Force on
    public static final int kLimelightLED = 0;

    // Sets camera mode. 0 = Vision processor, 1 = Driver Camera
    public static final int kLimelightMode = 0;

    // Sets streaming mode. 0 = Side-by-Side, 1 = PiP main, 2 = PiP secondary
    public static final int kLimelightStream = 0;

    // The default pipeline to stream
    public static final int kLimelightStartingPipeline = 1;

    // Pipeline latency in milliseconds
    public static final String kLimelightLatencyID = "tl";

    // Whether or not a valid target is found (0 or 1)
    public static final String kLimelightTargetID = "tv";

    // Horizontal offset from crosshair to target (+/- 27 degrees)
    public static final String kLimelightTargetXID = "tx";

    // Vertical offset from crosshair to target (+/- 20.5 degrees)
    public static final String kLimelightTargetYID = "ty";

    // Target area (0-100 % of image)
    public static final String kLimelightTargetAreaID = "ta";

    // Target skew/rotation (-90 to 0 degrees)
    public static final String kLimelightTargetSkewID = "ts";

    // Vertical sidelength of bounding box (0-320 pixels)
    public static final String kLimelightTargetVertID = "tvert";

    // Horizontal sidelength of bounding box (0-320 pixels)
    public static final String kLimelightTargetHorID = "thor";

    // Camera Variables
    public static final double kCameraHeight = 4;
    public static final double kCameraAngle = -31.47286489; // -28.23744554
    public static final double kTargetHeight = 31.5;
    public static final double kpAim = -0.02;
    public static final double kpDistance = -0.05;
    public static final double kmin_aim_command = -0.5;
    public static final double kdistance_command = -0.5;
    public static final double kTargetDistanceFromTarget = 24;

    // Constants for April Tags using PhotonVision below
    public static final double kFieldLength = Units.inchesToMeters(651.2);
    public static final double kFieldWidth = Units.inchesToMeters(315.75);

    // needs to be replaced with what's in http://photonvision:5800 settings
    public static final double camFOV = 0;
    public static final int camResWidth = 0;
    public static final int camResHeight = 0;
    // Set this to what we plan on using, probably should play around with a good distance.
    public static final double minClosestTargetDistance = 0; // Goes by meters

    // public static final AprilTagFieldLayout tagPlayground =
    //     new AprilTagFieldLayout(null, kFieldLength, kFieldWidth);

    public static Transform3d robotToCamera =
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static String kPhotonTargetPose;
    public static String kPhotonTargetXID;
  }
}
