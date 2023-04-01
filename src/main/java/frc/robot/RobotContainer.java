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
import frc.robot.commands.Arm.Dislocator.DislocatorTele;
import frc.robot.commands.PermAutos.ConeDropLowGrab;
import frc.robot.commands.PermAutos.CubeDropLowGrab;
import frc.robot.commands.PermAutos.Middle1DropHigh;
import frc.robot.commands.PermAutos.Middle1DropHighChase;
import frc.robot.commands.PermAutos.Middle1DropLowOut;
import frc.robot.commands.PermAutos.PotatoHigh;
import frc.robot.commands.PermAutos.PotatoLow;
import frc.robot.commands.PresetPoses.DropGroundPosition;
import frc.robot.commands.PresetPoses.DropHighBack;
import frc.robot.commands.PresetPoses.DropHighPosition;
import frc.robot.commands.PresetPoses.DropLowPosition;
import frc.robot.commands.PresetPoses.GrabBackPosition;
import frc.robot.commands.PresetPoses.GrabDropPosition;
import frc.robot.commands.PresetPoses.GrabPosition;
import frc.robot.commands.PresetPoses.GrabStationPosition;
import frc.robot.commands.PresetPoses.GrabStationPositionBack;
import frc.robot.commands.PresetPoses.TuckPosition;
import frc.robot.commands.TempAutos.DoNothin;
import frc.robot.commands.TempAutos.Middle1DropHighOut;
import frc.robot.commands.TempAutos.Middle1DropLow;
import frc.robot.commands.TempAutos.MiddleMoveOut;
import frc.robot.commands.TempAutos.Station1DropHigh;
import frc.robot.commands.TempAutos.Station1DropHighPark;
import frc.robot.commands.TempAutos.Station1DropLow;
import frc.robot.commands.TempAutos.Station2Drop;
import frc.robot.commands.TempAutos.StationMoveOut;
import frc.robot.commands.TempAutos.TestAuto;
import frc.robot.commands.TempAutos.Wall1DropHigh;
import frc.robot.commands.TempAutos.Wall1DropLow;
import frc.robot.commands.TempAutos.Wall1DropLowGrab;
import frc.robot.commands.TempAutos.WallMoveOut;
import frc.robot.commands.Travelator.TravelatorLeveling;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.commands.Travelator.TravelatorTele;
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
  private Joystick ButtonBoardLeft = new Joystick(Constants.kButtonBoardLeftID);
  private Joystick ButtonBoardRight = new Joystick(Constants.kButtonBoardRightID);

  // Joystick Buttons
  private Trigger presetDropMiddleBtn;
  private Trigger presetDropHighBtn;
  private Trigger presetDropHighBackBtn;
  private Trigger presetDropBottomBtn;
  private Trigger presetGrabBtn;
  private Trigger presetGrabBtn2;
  private Trigger presetGrabBackLow;
  private Trigger presetGrabDropBtn;
  private Trigger presetMoveBtn;
  private Trigger presetMoveBtn2;
  private Trigger presetStartBtn;
  private Trigger presetStationBtn;
  private Trigger presetStationBackBtn;
  private Trigger presetBalanceBtn;
  private Trigger manualDriveBtn;

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
  private Middle1DropHighOut cmdMiddle1DropHighOut =
      new Middle1DropHighOut(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Middle1DropLowOut cmdMiddle1DropLowOut =
      new Middle1DropLowOut(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  private Station1DropHighPark cmdStation1DropHighPark =
      new Station1DropHighPark(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  // private Middle1DropHighPark cmdMiddle1DropHighPark =
  //    new Middle1DropHighPark(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  private StationMoveOut cmdStationMoveOut =
      new StationMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private MiddleMoveOut cmdMiddleMoveOut =
      new MiddleMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Middle1DropHighChase cmdMiddle1DropHighChase =
      new Middle1DropHighChase(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private WallMoveOut cmdWallMoveOut =
      new WallMoveOut(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private PotatoLow cmdPotatoLow =
      new PotatoLow(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private PotatoHigh cmdPotatoHigh =
      new PotatoHigh(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  private ConeDropLowGrab cmdConeDropLowGrab =
      new ConeDropLowGrab(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private Wall1DropLowGrab cmdWall1DropLowGrab =
      new Wall1DropLowGrab(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private CubeDropLowGrab cmdCubeDropHighGrab =
      new CubeDropLowGrab(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  private Station2Drop cmdStation2Drop =
      new Station2Drop(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  private DoNothin cmdDoNothin =
      new DoNothin(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);
  private TestAuto cmdTestAuto =
      new TestAuto(sDrivetrain, sDislocator, sElbow, sShoulder, sWrist, sTravelator);

  // private LeftSingleCharger cmdLeftCharge = new LeftSingleCharger(sDrivetrain);
  // private MiddleSingleCharger cmdMidCharge = new MiddleSingleCharger(sDrivetrain);
  // private RightSingle cmdRightSing = new RightSingle(sDrivetrain);
  // private RightSingleCharger cmdRightCharge = new RightSingleCharger(sDrivetrain);
  // private Path1Double cmdP1Double = new Path1Double(sDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, OpController, false));
    sTravelator.setDefaultCommand(new TravelatorTele(sTravelator, ButtonBoardRight));
    sDislocator.setDefaultCommand(new DislocatorTele(sDislocator, ButtonBoardLeft));
    // sShoulder.setDefaultCommand(new ShoulderMoveToPositionTemp(sShoulder.getShoulderDesirePos(),
    // sShoulder));

    // Create auto chooser

    // Station Autos
    // AutoSelect.setDefaultOption("Stat1DropLow", cmdStation1DropLow);
    AutoSelect.addOption("Cone Cube", cmdConeDropLowGrab);
    AutoSelect.addOption("The High Ground", cmdCubeDropHighGrab);
    // AutoSelect.addOption("Station High Move", cmdStation1DropHigh);
    // AutoSelect.addOption("Stat1DropHighPark", cmdStation1DropHighPark);
    // AutoSelect.addOption("StatMoveOut", cmdStationMoveOut);

    // Middle Autos
    // AutoSelect.addOption("Mid1DropLow", cmdMiddle1DropLow);
    AutoSelect.addOption("Middle Drop High", cmdMiddle1DropHigh);
    // AutoSelect.addOption("Middle High Out", cmdMiddle1DropHighOut);
    AutoSelect.addOption("Middle Middle Out", cmdMiddle1DropLowOut);
    // AutoSelect.addOption("MidMoveOut", cmdMiddleMoveOut);

    // Wall Autos
    // AutoSelect.addOption("Wall1DropLow", cmdWall1DropLow);
    // AutoSelect.addOption("Wall1DropLowGrab", cmdWall1DropLowGrab);
    // AutoSelect.addOption("Wall1DropHigh", cmdWall1DropHigh);
    AutoSelect.addOption("Middle High Chase", cmdMiddle1DropHighChase);
    // AutoSelect.addOption("WallMoveOut", cmdWallMoveOut);

    // Potato Autos
    // AutoSelect.addOption("PotatoLow", cmdPotatoLow);
    // AutoSelect.addOption("PotatoHigh", cmdPotatoHigh);

    // Nothing Auto
    // AutoSelect.setDefaultOption("DoNothin", cmdDoNothin);

    // Test Autos
    // AutoSelect.addOption("TestAuto(DO NOT USE)", cmdTestAuto);

    // ShuffleBoard
    SmartDashboard.putData(AutoSelect); // Adds auto select to dashboard.
    SmartDashboard.putData("RESET DRIVE SENSORS", new ResetDriveSensors(sDrivetrain));
    SmartDashboard.putData(
        "Starting Position", new TuckPosition(sDislocator, sElbow, sShoulder, sTravelator));

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
    // fireParkingBrake.toggleOnTrue(new TravelatorLeveling(sTravelator, sDrivetrain));

    // fireParkingBrakeWork = new POVButton(OpController, 0);
    // fireParkingBrakeWork.onTrue(sDrivetrain.cmdToggleBrake());

    // fireParkingBrake.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack,
    // sTravelator));
    // fireParkingBrake.onTrue(new DislocatorMoveToPosition(0, sDislocator));
    // fireParkingBrake.onTrue(new ShoulderMoveToPosition(0, sShoulder));
    // fireParkingBrake.onTrue(new ElbowMoveToPosition(0, sElbow));
    // fireParkingBrake.onTrue(sWrist.cmdGrabOpen());

    // Driving modes

    // coneDriveBtn = new JoystickButton(OpController, 6);
    // coneDriveBtn.toggleOnTrue(new ConeDropOff(OpController, sDrivetrain, sLimeLight));

    // cubeDriveBtn = new JoystickButton(OpController, 5);
    // cubeDriveBtn.toggleOnTrue(new CubeDropOff(OpController, sDrivetrain, sPhotonVision));

    toggleGearBtn = new JoystickButton(OpController, 9);
    toggleGearBtn.onTrue(sDrivetrain.cmdToggleGear());

    manualDriveBtn = new JoystickButton(ButtonBoardRight, 9);
    manualDriveBtn.onTrue(new TravelatorTele(sTravelator, ButtonBoardRight));
    manualDriveBtn.onTrue(new DislocatorTele(sDislocator, ButtonBoardLeft));

    // Shoulder Buttons

    shoulderForwardBtn = new JoystickButton(ButtonBoardRight, 6);
    shoulderForwardBtn.whileTrue(sShoulder.cmdShoulderForward());

    shoulderBackwardBtn = new JoystickButton(ButtonBoardRight, 5);
    shoulderBackwardBtn.whileTrue(sShoulder.cmdShoulderBackward());

    // Travelator Buttons

    // travelatorForwardBtn = new JoystickButton(coopStick2, 5);
    // travelatorForwardBtn.whileTrue(sTravelator.cmdMoveForward());

    // travelatorBackwardBtn = new JoystickButton(coopStick2, 3);
    // travelatorBackwardBtn.whileTrue(sTravelator.cmdMoveBackward());

    // travelAutoFrontBtn = new POVButton(coopStick1, 0);
    // travelAutoFrontBtn.onTrue(
    //     new TravelatorMoveToPosition(opConstants.kTravelatorFront, sTravelator));

    // travelAutoBackBtn = new POVButton(coopStick1, 180);
    // travelAutoBackBtn.onTrue(
    //     new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));

    travelLevelingBtn = new JoystickButton(ButtonBoardLeft, 9);
    travelLevelingBtn.onTrue(new TravelatorLeveling(sTravelator, sDrivetrain));

    // Wrist Buttons

    grabBtn = new JoystickButton(ButtonBoardLeft, 10);
    grabBtn.onTrue(sWrist.GrabToggle());

    grabBtn = new JoystickButton(ButtonBoardRight, 10);
    grabBtn.onTrue(sWrist.GrabToggle());

    // Elbow Buttons

    ElbowForwardBtn = new JoystickButton(ButtonBoardLeft, 6);
    ElbowForwardBtn.whileTrue(sElbow.cmdArmElbowForward());

    ElbowBackwardBtn = new JoystickButton(ButtonBoardLeft, 5);
    ElbowBackwardBtn.whileTrue(sElbow.cmdArmElbowBackward());

    // Preset Buttons

    presetStartBtn = new JoystickButton(ButtonBoardLeft, 1);
    presetStartBtn.onTrue(sWrist.cmdResetWristEncoder());
    presetStartBtn.onTrue(sElbow.cmdResetElbowEncoder());
    presetStartBtn.onTrue(sShoulder.cmdResetShoulderEncoder());

    // Preset the robot to grab upright

    presetGrabBtn = new JoystickButton(ButtonBoardLeft, 4);

    // Numbers before Drey

    // presetGrabBtn.onTrue(new TravelatorMoveToPosition(2, sTravelator));
    // presetGrabBtn.onTrue(new DislocatorMoveToPosition(22, sDislocator));
    // presetGrabBtn.onTrue(new ShoulderMoveToPosition(-124, sShoulder));
    // presetGrabBtn.onTrue(new ElbowMoveToPositionTele(45, sElbow));

    presetGrabBtn.onTrue(new TravelatorMoveToPosition(2, sTravelator));
    presetGrabBtn.onTrue(new GrabPosition(sDislocator, sElbow, sShoulder, sTravelator));

    presetGrabBtn2 = new JoystickButton(OpController, 1);
    presetGrabBtn2.onTrue(new TravelatorMoveToPosition(2, sTravelator));
    presetGrabBtn2.onTrue(new GrabPosition(sDislocator, sElbow, sShoulder, sTravelator));

    presetGrabBackLow = new JoystickButton(ButtonBoardLeft, 8);
    presetGrabBackLow.onTrue(new TravelatorMoveToPosition(2, sTravelator));
    presetGrabBackLow.onTrue(new GrabBackPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Grab at Drop station button
    presetGrabDropBtn = new JoystickButton(ButtonBoardLeft, 3);
    presetGrabDropBtn.onTrue(new GrabDropPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Grab at station button

    presetStationBackBtn = new JoystickButton(ButtonBoardLeft, 2);
    presetStationBackBtn.onTrue(new GrabStationPositionBack(sDislocator, sElbow, sShoulder, sTravelator));

    presetStationBtn = new JoystickButton(ButtonBoardRight, 8);
    presetStationBtn.onTrue(new GrabStationPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Preset the robot to safe driving position

    presetMoveBtn = new JoystickButton(ButtonBoardLeft, 11);
    presetMoveBtn.onTrue(new TuckPosition(sDislocator, sElbow, sShoulder, sTravelator));

    presetMoveBtn2 = new JoystickButton(ButtonBoardRight, 11);
    presetMoveBtn2.onTrue(new TuckPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Preset the robot to drop a cone on the lower pole

    presetDropBottomBtn = new JoystickButton(ButtonBoardRight, 4);
    presetDropBottomBtn.onTrue(new DropGroundPosition(sDislocator, sElbow, sShoulder, sTravelator));

    presetDropMiddleBtn = new JoystickButton(ButtonBoardRight, 3);
    presetDropMiddleBtn.onTrue(new DropLowPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Preset the robot to drop a cone on the higher pole

    presetDropHighBtn = new JoystickButton(ButtonBoardRight, 2);
    presetDropHighBtn.onTrue(new DropHighPosition(sDislocator, sElbow, sShoulder, sTravelator));

    // Preset the robot to drop a cone on the higher pole from the back

    presetDropHighBackBtn = new JoystickButton(ButtonBoardLeft, 7);
    presetDropHighBackBtn.onTrue(new DropHighBack(sDislocator, sElbow, sShoulder, sTravelator));

    // Preset to start balancing on the Charge Station

    presetBalanceBtn = new JoystickButton(OpController, 4);
    presetBalanceBtn.onTrue(new TuckPosition(sDislocator, sElbow, sShoulder, sTravelator));
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

  public Drivetrain getDrivsetrain() {
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

  public Joystick getButtonBoardLeft() {
    return ButtonBoardLeft;
  }

  public Joystick getButtonBoardRight() {
    return ButtonBoardRight;
  }

  public XboxController getOpController() {
    return OpController;
  }
}
