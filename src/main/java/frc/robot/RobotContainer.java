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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.camConstants;
import frc.robot.Constants.opConstants;
import frc.robot.commands.Arm.Dislocator.DislocatorMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPosition;
import frc.robot.commands.Arm.Elbow.ElbowMoveToPositionTele;
import frc.robot.commands.Arm.Shoulder.ShoulderMoveToPosition;
import frc.robot.commands.PresetPoses.StartingPosition;
import frc.robot.commands.TempAutos.DoNothin;
import frc.robot.commands.TempAutos.Middle1DropHigh;
import frc.robot.commands.TempAutos.Middle1DropLow;
import frc.robot.commands.TempAutos.MiddleMoveOut;
import frc.robot.commands.TempAutos.PotatoHigh;
import frc.robot.commands.TempAutos.PotatoLow;
import frc.robot.commands.TempAutos.Station1DropHigh;
import frc.robot.commands.TempAutos.Station1DropLow;
import frc.robot.commands.TempAutos.StationMoveOut;
import frc.robot.commands.TempAutos.Wall1DropHigh;
import frc.robot.commands.TempAutos.Wall1DropLow;
import frc.robot.commands.TempAutos.WallMoveOut;
import frc.robot.commands.Travelator.TravelatorLeveling;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.commands.drivetrain.OperatorDrive;
import frc.robot.commands.drivetrain.ResetDriveSensors;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Grabber;
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
  private Grabber sGrabber = new Grabber();
  private Travelator sTravelator = new Travelator();

  //  Joysticks
  private XboxController OpController = new XboxController(Constants.kOpStickID);
  private Joystick coopStick1 = new Joystick(Constants.kCoopStick1ID);
  private Joystick coopStick2 = new Joystick(Constants.kCoopStick2ID);

  // Joystick Buttons
  private Trigger presetDropLowBtn;
  private Trigger presetDropHighBtn;
  private Trigger presetGrabBtn;
  private Trigger presetMoveBtn;
  private Trigger presetStartBtn;
  private Trigger presetStationBtn;
  private Trigger presetBalanceBtn;

  // Driving modes
  private Trigger coneDriveBtn;
  private Trigger cubeDriveBtn;
  private Trigger toggleGearBtn;

  // Shoulder
  private Trigger shoulderForwardBtn;
  private Trigger shoulderBackwardBtn;

  // Travelator
  private Trigger travelatorForwardBtn;
  private Trigger travelatorBackwardBtn;
  private Trigger travelAutoFrontBtn;
  private Trigger travelAutoBackBtn;
  private Trigger travelLevelingBtn;

  // Elbow
  private Trigger ElbowForwardBtn;
  private Trigger ElbowBackwardBtn;

  // Dislocator
  private Trigger DislocatorForwardBtn;
  private Trigger DislocatorBackwardBtn;

  // Misc
  private Trigger fireParkingBrake;
  private Trigger fireParkingBrakeWork;
  private Trigger grabBtn;

  // Auto Chooser
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autonomous

  private Station1DropLow cmdStation1DropLow =
      new Station1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private Middle1DropLow cmdMiddle1DropLow =
      new Middle1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private Wall1DropLow cmdWall1DropLow =
      new Wall1DropLow(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private Station1DropHigh cmdStation1DropHigh =
      new Station1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private Middle1DropHigh cmdMiddle1DropHigh =
      new Middle1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private Wall1DropHigh cmdWall1DropHigh =
      new Wall1DropHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private StationMoveOut cmdStationMoveOut =
      new StationMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private MiddleMoveOut cmdMiddleMoveOut =
      new MiddleMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private WallMoveOut cmdWallMoveOut =
      new WallMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private PotatoLow cmdPotatoLow =
      new PotatoLow(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private PotatoHigh cmdPotatoHigh =
      new PotatoHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);
  private DoNothin cmdDoNothin =
      new DoNothin(sDrivetrain, sDislocator, sElbow, sShoulder, sGrabber, sTravelator);

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
    AutoSelect.addOption("StationMoveOut", cmdStationMoveOut);
    AutoSelect.addOption("MiddleMoveOut", cmdMiddleMoveOut);
    AutoSelect.addOption("WallMoveOut", cmdWallMoveOut);
    AutoSelect.addOption("PotatoLow", cmdPotatoLow);
    AutoSelect.addOption("PotatoHigh", cmdPotatoHigh);
    AutoSelect.addOption("DoNothin", cmdDoNothin);

    // ShuffleBoard
    SmartDashboard.putData(AutoSelect); // Adds auto select to dashboard.
    SmartDashboard.putData("RESET DRIVE SENSORS", new ResetDriveSensors(sDrivetrain));
    SmartDashboard.putData(
        "Starting Position", new StartingPosition(sDislocator, sElbow, sShoulder, sGrabber));

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

    fireParkingBrake = new POVButton(OpController, 180);
    fireParkingBrake.onTrue(sDrivetrain.cmdToggleBrake());


    toggleGearBtn = new JoystickButton(OpController, 9);
    toggleGearBtn.onTrue(sDrivetrain.cmdToggleGear());

    // Shoulder Buttons

    shoulderForwardBtn = new JoystickButton(coopStick2, 4);
    shoulderForwardBtn.whileTrue(sShoulder.cmdShoulderForward());

    shoulderBackwardBtn = new JoystickButton(coopStick2, 6);
    shoulderBackwardBtn.whileTrue(sShoulder.cmdShoulderBackward());

    // Travelator Buttons

    travelatorForwardBtn = new JoystickButton(coopStick2, 5);
    travelatorForwardBtn.whileTrue(sTravelator.cmdMoveForward());

    travelatorBackwardBtn = new JoystickButton(coopStick2, 3);
    travelatorBackwardBtn.whileTrue(sTravelator.cmdMoveBackward());

    travelLevelingBtn = new JoystickButton(coopStick1, 11);
    travelLevelingBtn.onTrue(new TravelatorLeveling(sTravelator, sDrivetrain));

    // Grab Buttons

    grabBtn = new JoystickButton(coopStick2, 1);
    grabBtn.onTrue(sGrabber.GrabToggle());

    // Elbow Buttons

    ElbowForwardBtn = new JoystickButton(coopStick1, 6);
    ElbowForwardBtn.whileTrue(sElbow.cmdArmElbowForward());

    ElbowBackwardBtn = new JoystickButton(coopStick1, 4);
    ElbowBackwardBtn.whileTrue(sElbow.cmdArmElbowBackward());

    // Dislocate your arm!

    DislocatorForwardBtn = new JoystickButton(coopStick1, 5);
    DislocatorForwardBtn.whileTrue(sDislocator.cmdDislocatorMoveForward());

    DislocatorBackwardBtn = new JoystickButton(coopStick1, 3);
    DislocatorBackwardBtn.whileTrue(sDislocator.cmdDislocatorMoveBackward());

    // Preset Buttons

    presetStartBtn = new JoystickButton(coopStick1, 7);
    presetStartBtn.onTrue(sElbow.cmdResetElbowEncoder());
    presetStartBtn.onTrue(sShoulder.cmdResetShoulderEncoder());

    // Preset the robot to grab upright

    presetGrabBtn = new JoystickButton(OpController, 3);
    presetGrabBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    presetGrabBtn.onTrue(new DislocatorMoveToPosition(17, sDislocator));
    presetGrabBtn.onTrue(new ShoulderMoveToPosition(-460, sShoulder));
    presetGrabBtn.onTrue(new ElbowMoveToPositionTele(-39, sElbow));

    // Grab at station button

    presetStationBtn = new JoystickButton(coopStick1, 1);
    presetStationBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack + 3, sTravelator));
    presetStationBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetStationBtn.onTrue(new ShoulderMoveToPosition(-165, sShoulder));
    presetStationBtn.onTrue(new ElbowMoveToPositionTele(65, sElbow));

    // Preset the robot to safe driving position

    presetMoveBtn = new JoystickButton(OpController, 2);
    presetMoveBtn.onTrue(new ShoulderMoveToPosition(-183, sShoulder));
    presetMoveBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    presetMoveBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));

    // Preset the robot to drop a cone on the lower pipe

    presetDropLowBtn = new JoystickButton(OpController, 1);
    presetDropLowBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorFront - 3.5, sTravelator));
    presetDropLowBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetDropLowBtn.onTrue(new ShoulderMoveToPosition(-199.5, sShoulder));
    presetDropLowBtn.onTrue(new ElbowMoveToPositionTele(30, sElbow));

    // Preset the robot to drop a cone on the higher pipe

    presetDropHighBtn = new JoystickButton(OpController, 4);
    presetDropHighBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorFront, sTravelator));
    presetDropHighBtn.onTrue(new DislocatorMoveToPosition(22, sDislocator));
    presetDropHighBtn.onTrue(new ShoulderMoveToPosition(-150, sShoulder));
    presetDropHighBtn.onTrue(new ElbowMoveToPositionTele(30, sElbow));

    // Preset to start balancing on the Charge Station

    presetBalanceBtn = new JoystickButton(coopStick2, 2);
    presetBalanceBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorMiddle, sTravelator));
    presetBalanceBtn.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    presetBalanceBtn.onTrue(new ShoulderMoveToPosition(-330, sShoulder));
    presetBalanceBtn.onTrue(new ElbowMoveToPositionTele(90, sElbow));
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

  public Grabber getGrabber() {
    return sGrabber;
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
