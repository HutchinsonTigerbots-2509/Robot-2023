// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.opConstants;
import frc.robot.commands.OrientalDrive;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.commands.Wrist.WristMoveToPosition;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.RightSingle;
import frc.robot.commands.drivetrain.ResetDriveSensors;
import frc.robot.subsystems.Arms.Dislocator;
import frc.robot.subsystems.Arms.Elbow;
import frc.robot.subsystems.Arms.Shoulder;
import frc.robot.subsystems.Arms.Wrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;
import frc.robot.subsystems.Vision.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drivetrain sDrivetrain = new Drivetrain();
  private LimeLight sLimeLight = new LimeLight();
  private Shoulder sShoulder = new Shoulder();
  private Dislocator sDislocator = new Dislocator();
  private Elbow sElbow = new Elbow();
  private Wrist sWrist = new Wrist();
  private Travelator sTravelator = new Travelator();

  //  Joysticks
  private Joystick coopStick = new Joystick(Constants.kCoopStickID);
  private Joystick opStick = new Joystick(Constants.kOpStickID);

  // Joystick Buttons
  private Trigger shoulderForwardBtn;
  private Trigger shoulderBackwardBtn;
  private Trigger travelatorForwardBtn;
  private Trigger travelatorBackwardBtn;
  private Trigger travelatorBackPosBtn;
  private Trigger travelatorMiddlePosBtn;
  private Trigger travelatorFrontPosBtn;
  private Trigger armWristForwardBtn;
  private Trigger armWristBackwardBtn;
  private Trigger armWristZeroBtn;
  private Trigger armWristNinetyBtn;
  private Trigger armElbowForwardBtn;
  private Trigger armElbowBackwardBtn;
  private Trigger fireParkingBrake;
  private Trigger grabBtn;
  private Trigger DislocatorForwardBtn;
  private Trigger DislocatorBackwardBtn;

  // Auto Chooser
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autonomous
  private Potato cmdPotato = new Potato(sDrivetrain);
  private LeftSingleCharger cmdLeftCharge = new LeftSingleCharger(sDrivetrain);
  private MiddleSingleCharger cmdMidCharge = new MiddleSingleCharger(sDrivetrain);
  private RightSingle cmdRightSing = new RightSingle(sDrivetrain);
  private RightSingleCharger cmdRightCharge = new RightSingleCharger(sDrivetrain);
  private Path1Double cmdP1Double = new Path1Double(sDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OrientalDrive(opStick, sDrivetrain));

    // Create auto chooser
    AutoSelect.setDefaultOption("Potato", cmdPotato);
    AutoSelect.addOption("P1", cmdP1Double); // Replaced Left Single
    AutoSelect.addOption("Left Charge", cmdLeftCharge);
    AutoSelect.addOption("Mid Charge", cmdMidCharge);
    AutoSelect.addOption("Right Single", cmdRightSing);
    AutoSelect.addOption("Right Charge", cmdRightCharge);

    // ShuffleBoard
    SmartDashboard.putData(AutoSelect); // Adds auto select to dashboard.
    SmartDashboard.putData("RESET DRIVE SENSORS", new ResetDriveSensors(sDrivetrain));

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
    fireParkingBrake = new JoystickButton(opStick, 2);
    fireParkingBrake.onTrue(sDrivetrain.cmdToggleBrake());

    // Shoulder Buttons
    shoulderForwardBtn = new JoystickButton(coopStick, 7);
    shoulderForwardBtn.whileTrue(sShoulder.cmdArmLiftForward());
    shoulderBackwardBtn = new JoystickButton(coopStick, 8);
    shoulderBackwardBtn.whileTrue(sShoulder.cmdArmLiftBackward());

    // Travelator Buttons
    travelatorForwardBtn = new JoystickButton(opStick, 7);
    travelatorForwardBtn.whileTrue(sTravelator.cmdMoveForward());
    travelatorBackwardBtn = new JoystickButton(opStick, 11);
    travelatorBackwardBtn.whileTrue(sTravelator.cmdMoveBackward());

    travelatorBackPosBtn = new JoystickButton(opStick, 12);
    travelatorBackPosBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));
    travelatorMiddlePosBtn = new JoystickButton(opStick, 10);
    travelatorMiddlePosBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorMiddle, sTravelator));
    travelatorFrontPosBtn = new JoystickButton(opStick, 8);
    travelatorFrontPosBtn.onTrue(
        new TravelatorMoveToPosition(opConstants.kTravelatorFront, sTravelator));

    // Wrist Buttons
    armWristForwardBtn = new POVButton(coopStick, 90);
    armWristForwardBtn.whileTrue(sElbow.cmdArmElbowForward());
    armWristBackwardBtn = new POVButton(coopStick, 270);
    armWristBackwardBtn.whileTrue(sElbow.cmdArmElbowBackward());
    armWristZeroBtn = new POVButton(coopStick, 0);
    armWristZeroBtn.whileTrue(new WristMoveToPosition(0, sWrist));
    armWristNinetyBtn = new POVButton(coopStick, 180);
    armWristNinetyBtn.whileTrue(new WristMoveToPosition(90, sWrist));
    grabBtn = new JoystickButton(coopStick, 1);
    grabBtn.onTrue(sWrist.Grab());

    // Elbow Buttons
    armElbowForwardBtn = new JoystickButton(coopStick, 6);
    armElbowForwardBtn.whileTrue(sElbow.cmdArmElbowForward());
    armElbowBackwardBtn = new JoystickButton(coopStick, 4);
    armElbowBackwardBtn.whileTrue(sElbow.cmdArmElbowBackward());

    // Dislocate your arm!
    DislocatorForwardBtn = new JoystickButton(coopStick, 0);
    DislocatorForwardBtn.whileTrue(sDislocator.cmdDislocatorMoveForward());
    DislocatorBackwardBtn = new JoystickButton(coopStick, 0);
    DislocatorBackwardBtn.whileTrue(sDislocator.cmdDislocatorMoveBackward());
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

  /**
   * Gets the {@link Drivetrain} subsystem.
   *
   * @return {@link Drivetrain}
   */
  public Drivetrain getDrivetrain() {
    return sDrivetrain;
  }

  /**
   * Gets the {@link Shoulder} subsystem.
   *
   * @return {@link Shoulder}
   */
  public Shoulder getShoulder() {
    return sShoulder;
  }

  /**
   * Gets the {@link Dislocator} subsystem.
   *
   * @return {@link Dislocator}
   */
  public Dislocator getDislocator() {
    return sDislocator;
  }

  /**
   * Gets the {@link Elbow} subsystem.
   *
   * @return {@link Elbow}
   */
  public Elbow getElbow() {
    return sElbow;
  }

  /**
   * Gets the {@link Wrist} subsystem.
   *
   * @return {@link Wrist}
   */
  public Wrist getWrist() {
    return sWrist;
  }

  /**
   * Gets the {@link Travelator} subsystem.
   *
   * @return {@link Travelator}
   */
  public Travelator gTravelator() {
    return sTravelator;
  }

  /**
   * Gets the {@link LimeLight} subsystem.
   *
   * @return {@link LimeLight}
   */
  public LimeLight getLimeLight() {
    return sLimeLight;
  }

  /**
   * Gets the Co-Driver {@link Joystick}
   *
   * @return {@link Joystick}
   */
  public Joystick getCoopStick() {
    return coopStick;
  }

  /**
   * Gets the Driver {@link Joystick}
   *
   * @return {@link Joystick}
   */
  public Joystick getOpStick() {
    return opStick;
  }
}
