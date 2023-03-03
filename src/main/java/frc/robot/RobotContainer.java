// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.camConstants;
import frc.robot.Constants.opConstants;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.Arm.Wrist.WristMoveToPosition;
import frc.robot.commands.PresetPoses.StartingPosition;
import frc.robot.commands.TempAutos.Middle1DropHigh;
import frc.robot.commands.TempAutos.Middle1DropLow;
import frc.robot.commands.TempAutos.Station1DropHigh;
import frc.robot.commands.TempAutos.Station1DropLow;
import frc.robot.commands.TempAutos.Wall1DropHigh;
import frc.robot.commands.TempAutos.Wall1DropLow;
import frc.robot.commands.Travelator.TravelatorLeveling;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.commands.drivetrain.ConeDropOff;
import frc.robot.commands.drivetrain.CubeDropOff;
import frc.robot.commands.drivetrain.DrivetrainBalancing;
import frc.robot.commands.drivetrain.OperatorDrive;
import frc.robot.commands.drivetrain.ResetDriveSensors;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.subsystems.Vision.PhotonVision;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static AprilTagFieldLayout aprilTagField =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(14.25),
                      Units.inchesToMeters(265.74),
                      Units.inchesToMeters(27.38),
                      new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(147.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(108.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(42.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0))))),
          camConstants.kFieldLength,
          camConstants.kFieldWidth);

  // Subsystems
  private Drivetrain sDrivetrain = new Drivetrain();
  private LimeLight sLimeLight = new LimeLight();
  private PhotonVision sPhotonVision = new PhotonVision();
  private Shoulder sShoulder = new Shoulder();
  private Dislocator sDislocator = new Dislocator();
  private Elbow sElbow = new Elbow();
  private Wrist sWrist = new Wrist();
  private Travelator sTravelator = new Travelator();

  //  Joysticks
  private XboxController OpController = new XboxController(Constants.kOpStickID);
  private Joystick coopStick1 = new Joystick(Constants.kCoopStick1ID);
  private Joystick coopStick2 = new Joystick(Constants.kCoopStick2ID);

  // Joystick Buttons
  private Trigger presetDropLowBtn;
  private Trigger presetDropHighBtn;
  private Trigger presetGrabBtn;
  private Trigger presetZeroBtn;
  private Trigger presetMoveBtn;
  private Trigger presetStartBtn;

  // Driving modes
  private Trigger coneDriveBtn;
  private Trigger cubeDriveBtn;

  // Shoulder
  private Trigger shoulderForwardBtn;
  private Trigger shoulderBackwardBtn;
  private Trigger shoulder0Btn;

  // Travelator
  private Trigger travelatorForwardBtn;
  private Trigger travelatorBackwardBtn;
  private Trigger travelatorBackPosBtn;
  private Trigger travelatorMiddlePosBtn;
  private Trigger travelatorFrontPosBtn;

  // Wrist
  private Trigger armWristForwardBtn;
  private Trigger armWristBackwardBtn;
  private Trigger armWristZeroBtn;
  private Trigger armWristNinetyBtn;

  // Elbow
  private Trigger ElbowForwardBtn;
  private Trigger ElbowBackwardBtn;
  private Trigger Elbow0Btn;

  // Dislocator
  private Trigger DislocatorForwardBtn;
  private Trigger DislocatorBackwardBtn;
  private Trigger DislocatorPresetBtn;

  // Misc
  private Trigger fireParkingBrake;
  private Trigger grabBtn;
  private Trigger balanceBtn;

  // Auto Chooser
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autonomous

  private Station1DropLow cmdStation1DropLow =
      new Station1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Middle1DropLow cmdMiddle1DropLow =
      new Middle1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Wall1DropLow cmdWall1DropLow =
      new Wall1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Station1DropHigh cmdStation1DropHigh =
      new Station1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Middle1DropHigh cmdMiddle1DropHigh =
      new Middle1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Wall1DropHigh cmdWall1DropHigh =
      new Wall1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  // private LeftSingleCharger cmdLeftCharge = new LeftSingleCharger(sDrivetrain);
  // private MiddleSingleCharger cmdMidCharge = new MiddleSingleCharger(sDrivetrain);
  // private RightSingle cmdRightSing = new RightSingle(sDrivetrain);
  // private RightSingleCharger cmdRightCharge = new RightSingleCharger(sDrivetrain);
  // private Path1Double cmdP1Double = new Path1Double(sDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, OpController, false));
    // sShoulder.setDefaultCommand(new ShoulderMoveToPositionTemp(sShoulder.getShoulderDesirePos(),
    // sShoulder));

    // Create auto chooser

    AutoSelect.setDefaultOption("Station1DropLow", cmdStation1DropLow);
    AutoSelect.addOption("Middle1DropLow", cmdMiddle1DropLow);
    AutoSelect.addOption("Wall1DropLow", cmdWall1DropLow);
    AutoSelect.addOption("Station1DropHigh", cmdStation1DropHigh);
    AutoSelect.addOption("Middle1DropHigh", cmdMiddle1DropHigh);
    AutoSelect.addOption("Wall1DropHigh", cmdWall1DropHigh);

    // ShuffleBoard
    SmartDashboard.putData(AutoSelect); // Adds auto select to dashboard.
    SmartDashboard.putData("RESET DRIVE SENSORS", new ResetDriveSensors(sDrivetrain));
    SmartDashboard.putData(
        "Starting Position", new StartingPosition(sDislocator, sElbow, sShoulder, sWrist));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Parking Break Buttons

    fireParkingBrake = new JoystickButton(OpController, 10);
    fireParkingBrake.onTrue(sDrivetrain.cmdToggleBrake());
    fireParkingBrake.toggleOnTrue(new TravelatorLeveling(sTravelator, sDrivetrain));
    fireParkingBrake.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    fireParkingBrake.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    fireParkingBrake.onTrue(new ShoulderMoveToPosition(0, sShoulder));
    fireParkingBrake.onTrue(new ElbowMoveToPosition(0, sElbow));
    fireParkingBrake.onTrue(sWrist.cmdGrabOpen());

    
    // balanceBtn = new JoystickButton(coopStick1, 12);
    // balanceBtn.toggleOnTrue(new DrivetrainBalancing(OpController, sDrivetrain));


    // Driving modes

    coneDriveBtn = new JoystickButton(OpController, 6);
    coneDriveBtn.toggleOnTrue(new ConeDropOff(OpController, sDrivetrain, sLimeLight));

    cubeDriveBtn = new JoystickButton(OpController, 5);
    cubeDriveBtn.toggleOnTrue(new CubeDropOff(OpController, sDrivetrain, sPhotonVision));

    // Shoulder Buttons

    shoulderForwardBtn = new JoystickButton(coopStick2, 6);
    shoulderForwardBtn.whileTrue(sShoulder.cmdShoulderForward());
    // shoulderForwardBtn.whileTrue(sShoulder.cmdShoulderPoseForward());

    shoulderBackwardBtn = new JoystickButton(coopStick2, 4);
    shoulderBackwardBtn.whileTrue(sShoulder.cmdShoulderBackward());
    // shoulderBackwardBtn.whileTrue(sShoulder.cmdShoulderPoseBackward());

    // Travelator Buttons

    travelatorForwardBtn = new JoystickButton(coopStick2, 5);
    travelatorForwardBtn.whileTrue(sTravelator.cmdMoveForward());

    travelatorBackwardBtn = new JoystickButton(coopStick2, 3);
    travelatorBackwardBtn.whileTrue(sTravelator.cmdMoveBackward());

    // Wrist Buttons

    armWristForwardBtn = new POVButton(coopStick2, 90);
    armWristForwardBtn.whileTrue(sWrist.cmdWristForward());

    armWristBackwardBtn = new POVButton(coopStick2, 270);
    armWristBackwardBtn.whileTrue(sWrist.cmdWristBackward());

    armWristZeroBtn = new POVButton(coopStick2, 0);
    armWristZeroBtn.onTrue(new WristMoveToPosition(0, sWrist));

    armWristNinetyBtn = new POVButton(coopStick2, 180);
    armWristNinetyBtn.onTrue(new WristMoveToPosition(90, sWrist));

    grabBtn = new JoystickButton(coopStick2, 1);
    grabBtn.onTrue(sWrist.GrabToggle());

    // Elbow Buttons

    ElbowForwardBtn = new JoystickButton(coopStick1, 6);
    ElbowForwardBtn.whileTrue(sElbow.cmdArmElbowForward());

    ElbowBackwardBtn = new JoystickButton(coopStick1, 4);
    ElbowBackwardBtn.whileTrue(sElbow.cmdArmElbowBackward());

    // Elbow0Btn = new JoystickButton(coopStick1, 11);
    // Elbow0Btn.onTrue(new ElbowMoveToPosition(0, sElbow));

    // Dislocate your arm!

    DislocatorForwardBtn = new JoystickButton(coopStick1, 5);
    DislocatorForwardBtn.whileTrue(sDislocator.cmdDislocatorMoveForward());

    DislocatorBackwardBtn = new JoystickButton(coopStick1, 3);
    DislocatorBackwardBtn.whileTrue(sDislocator.cmdDislocatorMoveBackward());

    // Preset to set up for a come standing up on the ground

    // Preset the robot to zero out
    presetZeroBtn = new JoystickButton(OpController, 1);
    presetZeroBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    presetZeroBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetZeroBtn.onTrue(new ShoulderMoveToPosition(0, sShoulder));
    presetZeroBtn.onTrue(new ElbowMoveToPosition(0, sElbow));
    presetZeroBtn.onTrue(new WristMoveToPosition(0, sWrist));

    // presetZeroBtn = new JoystickButton(coopStick2, 1);
    // presetZeroBtn.onTrue(new ZeroPosition(sDislocator, sElbow, sShoulder, sWrist));

    // Preset the robot to grab upright
    presetGrabBtn = new JoystickButton(OpController, 2);
    presetGrabBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    presetGrabBtn.onTrue(new DislocatorMoveToPosition(-15, sDislocator));
    presetGrabBtn.onTrue(new ShoulderMoveToPosition(-490, sShoulder));
    presetGrabBtn.onTrue(new ElbowMoveToPosition(-44, sElbow));
    presetGrabBtn.onTrue(new WristMoveToPosition(90, sWrist));

    // presetGrabBtn = new JoystickButton(coopStick2, 3);
    // presetGrabBtn.onTrue(new GrabPosition(sDislocator, sElbow, sShoulder, sWrist));

    // Preset the robot to safe driving position
    presetMoveBtn = new JoystickButton(OpController, 3);
    presetMoveBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    presetMoveBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetMoveBtn.onTrue(new ShoulderMoveToPosition(-200, sShoulder));
    presetMoveBtn.onTrue(new ElbowMoveToPosition(154, sElbow));
    presetMoveBtn.onTrue(new WristMoveToPosition(0, sWrist));


    // presetMoveBtn = new JoystickButton(coopStick2, 4);
    // presetMoveBtn.onTrue(new SafetyPosition(sDislocator, sElbow, sShoulder, sWrist));

    // Preset the robot to drop a cone on the lower pipe
    presetDropLowBtn = new JoystickButton(OpController, 4);
    presetDropLowBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorFront, sTravelator));
    presetDropLowBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetDropLowBtn.onTrue(new ShoulderMoveToPosition(-199.5, sShoulder));
    presetDropLowBtn.onTrue(new ElbowMoveToPosition(13.2, sElbow));
    presetDropLowBtn.onTrue(new WristMoveToPosition(13.6, sWrist));

    // presetDropLowBtn = new JoystickButton(coopStick2, 5);
    // presetDropLowBtn.onTrue(new DropLowPosition(sDislocator, sElbow, sShoulder, sWrist));

    // Preset the robot to drop a cone on the higher pipe
    // presetDropHighBtn = new JoystickButton(OpController, 1);
    // presetDropHighBtn.onTrue(
    //     new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    // presetDropHighBtn.onTrue(new DislocatorMoveToPosition(-22, sDislocator));
    // presetDropHighBtn.onTrue(new ShoulderMoveToPosition(-199.5, sShoulder));
    // presetDropHighBtn.onTrue(new ElbowMoveToPosition(27.5, sElbow));
    // presetDropHighBtn.onTrue(new WristMoveToPosition(0, sWrist));

    // presetDropHighBtn = new JoystickButton(coopStick2, 6);
    // presetDropHighBtn.onTrue(new DropHighPosition(sDislocator, sElbow, sShoulder, sWrist));

  }

  // Getter Methods
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoSelect.getSelected();
  }

  public Command getAutCommand() {
    return AutoSelect.getSelected();
  }

  // Getter Methods

  public Drivetrain getDrivetrain() {
    return sDrivetrain;
  }

  public Shoulder getShoulder() {
    return sShoulder;
  }

  public Wrist getWrist() {
    return sWrist;
  }

  public LimeLight getLimeLight() {
    return sLimeLight;
  }

  public PhotonVision getPhotonVision() {
    return sPhotonVision;
  }

  public Joystick getStick() {
    return coopStick2;
  }

  public Joystick getController() {
    return coopStick1;
  }

  public XboxController getOpController() {
    return OpController;
  }
}
