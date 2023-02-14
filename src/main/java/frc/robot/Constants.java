// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

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
    public final static int kOpStickID = 0;
    public final static int kCoopStickID = 1;

    public static final class opConstants {


    // ***** Drivetrain Constants ***** //
    public static final int kFrontLeftID = 14;
    public static final int kFrontRightID = 1;
    public static final int kRearLeftID = 15;
    public static final int kRearRightID = 0;
    public static final double kMaxSpeed = 0.8;
    public static final double kLowSpeed = 0.3;
    public static final double kMaxAngularSpeed = 0.8;
    public static final double kHighSpeedStrafe = 1;
    public static final double kLowSpeedStrafe = .6;
    public static final double kSkewRateLimit = 0.8;
    public static final double kGearRatio = 12.0;
    public static final int kFalconUnitsPerRotation = 2048;
    public static final double kWheelDiameter = 15.24; // This is in centimeters.
    public static final int kParkingBrakeExtend = 2;
    public static final int kParkingBrakeRetract = 3;


        // ***** Travelator Constants ***** //
        public final static int kTravelatorID = 13; 
        public final static double kTravelatorSpeed = 1;
        public final static int kBackRightLimitSwitchID = 0;
        public final static int kBackLeftLimitSwitchID = 1;
        public final static int kFrontRightLimitSwitchID = 2;
        public final static int kFrontLeftLimitSwitchID = 3;
        public final static double kTravelatorGearRatio = 29.118;
        public final static int kTravelatorMaxTicks = 291952;
        public final static int kTravelatorMinTicks = 0;

        // ***** Arm Constants ***** //
        public final static int kArmLiftID = 3;
        public final static int kGrabberP1 = 2;
        public final static int kGrabberP2 = 3;
        public final static int kArmCounterID = 0;
        public final static int kArmKnuckleID = 10;
        public final static int kArmWristID = 11;
        public final static double kMaxArmSpeed = .75;
    }


    public static final class camConstants {


        //#region ***** Vision Constants ***** //

        // Network
        public final static String kLimelightIP = "10.25.9.11";        // IP Address of Camera
        public final static String kLimelightNetworkID = "limelight";  // Name of Camera on Network
        
        // Settings
        public final static int kLimelightLED = 0;                     // Sets LED. 0 = Set by Pipline, 1 = Force off, 2 = Force blink, 3 = Force on
        public final static int kLimelightMode = 0;                    // Sets camera mode. 0 = Vision processor, 1 = Driver Camera
        public final static int kLimelightStream = 0;                  // Sets streaming mode. 0 = Side-by-Side, 1 = PiP main, 2 = PiP secondary
        public final static int kLimelightStartingPipeline = 1;        // The default pipeline to stream

        // Table IDs (for getting values from the Network Table)
        public final static String kLimelightLatencyID = "tl";         // Pipeline latency in milliseconds
        public final static String kLimelightTargetID = "tv";          // Whether or not a valid target is found (0 or 1)
        public final static String kLimelightTargetXID = "tx";         // Horizontal offset from crosshair to target (+/- 27 degrees)
        public final static String kLimelightTargetYID = "ty";         // Vertical offset from crosshair to target (+/- 20.5 degrees)
        public final static String kLimelightTargetAreaID = "ta";      // Target area (0-100 % of image)
        public final static String kLimelightTargetSkewID = "ts";      // Target skew/rotation (-90 to 0 degrees)
        public final static String kLimelightTargetVertID = "tvert";   // Vertical sidelength of bounding box (0-320 pixels)
        public final static String kLimelightTargetHorID = "thor";     // Horizontal sidelength of bounding box (0-320 pixels)

        // Camera Variables
        public final static double kCameraHeight = 4;
        //public static double kCameraAngle = -28.23744554;
        public final static double kCameraAngle = -31.47286489;
        public final static double kTargetHeight = 31.5;
        public final static double kpAim = -0.02;
        public final static double kpDistance = -0.05;
        public final static double kmin_aim_command = -0.5;
        public final static double kdistance_command = -0.5;
        public final static double kTargetDistanceFromTarget = 24;
        //#endregion

        // Constants for April Tags using PhotonVision below

        private static final double kFieldLength = Units.inchesToMeters(651.2);
        private static final double kFieldWidth = Units.inchesToMeters(315.75);

        public static final double camFOV = 0; // needs to be replaced with what's in http://photonvision:5800 settings
        public static final int camResWidth = 0; // "
        public static final int camResHeight = 0; // "
        public static final double minClosestTargetDistance = 0; // Set this to what we plan on using, probably should play around with a good distance. Goes by meters

        public static final AprilTagFieldLayout tagPlayground = new AprilTagFieldLayout(null, kFieldLength, kFieldWidth);
        
        public static Transform3d robotToCamera = new Transform3d(
            new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)
        );
        
    }

    public static final class ctrlConstants {


        //#region ***** Xbox Controller Constants ***** //
        public final static int kXboxLeftJoystickX = 0;
        public final static int kXboxLeftJoystickY = 1;
        
        public final static int kXboxRightJoystickX = 4;
        public final static int kXboxRightJoystickY = 5;
        
        public final static int kXboxLeftJoystickButton = 9;
        public final static int kXboxRightJoystickButton = 10;

        public final static int kXboxLeftTrigger = 2;
        public final static int kXboxRightTrigger = 3;
        
        public final static int kXboxLeftBumper = 5;
        public final static int kXboxRightBumper = 6;
        
        public final static int kXboxButtonA = 1;
        public final static int kXboxButtonB = 2;
        public final static int kXboxButtonX = 3;
        public final static int kXboxButtonY = 4;
        
        
        
        public final static int kXboxButtonBack = 7;
        public final static int kXboxButtonStart = 8;
        

        //#region ***** Joystick Buttons ***** //
        public final static int kJoystickX = 0;
        public final static int kJoystickY = 1;
        public final static int kJoystickZ = 2;

        public final static int kJoystickSlider = 3;

        public final static int kJoystickButton1 = 1;
        public final static int kJoystickButton2 = 2;
        public final static int kJoystickButton3 = 3;
        public final static int kJoystickButton4 = 4;
        public final static int kJoystickButton5 = 5;
        public final static int kJoystickButton6 = 6;
        public final static int kJoystickButton7 = 7;
        public final static int kJoystickButton8 = 8;
        public final static int kJoystickButton9 = 9;
        public final static int kJoystickButton10 = 10;
        public final static int kJoystickButton11 = 11;
        public final static int kJoystickButton12 = 12;
    }
}
