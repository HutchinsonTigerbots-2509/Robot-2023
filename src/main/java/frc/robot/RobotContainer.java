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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTele;
import frc.robot.commands.DriveVision;
import frc.robot.commands.OrientalDrive;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.RightSingle;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.drivetrain.ResetDriveSensors;
import frc.robot.subsystems.Arm;
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
  private Arm sArm = new Arm();
  private Travelator sTravelator = new Travelator();

  // Auto Chooser
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autonomous
  private Potato cmdPotato = new Potato(sDrivetrain);
  private LeftSingleCharger cmdLeftCharge = new LeftSingleCharger(sDrivetrain);
  private MiddleSingleCharger cmdMidCharge = new MiddleSingleCharger(sDrivetrain);
  private RightSingle cmdRightSing = new RightSingle(sDrivetrain);
  private RightSingleCharger cmdRightCharge = new RightSingleCharger(sDrivetrain);
  private Path1Double cmdP1Double = new Path1Double(sDrivetrain);

  // Joysticks
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  private CommandXboxController controller = new CommandXboxController(Constants.kOpStickID);

  // Joystick Buttons
  private Trigger turnToZero;
  // private Trigger driveToZero;
  private Trigger extendParkingBrake;
  private Trigger retractParkingBrake;
  private Trigger travelatorBackward;
  private Trigger travelatorForward;
  private JoystickButton arm1Btn;
  private JoystickButton arm1OutBtn;
  private JoystickButton arm2Btn;
  private JoystickButton arm2OutBtn;
  private JoystickButton armGrabBtn;
  private JoystickButton travelatorOutBtn;
  private JoystickButton travelatorInBtn;
  private JoystickButton autoVisionBtn;
  private JoystickButton gearBtn;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    sDrivetrain.setDefaultCommand(new OperatorDrive(sDrivetrain, stick, true));

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
    turnToZero = new JoystickButton(stick, 1);
    turnToZero.whileTrue(new RotateToAngle(0, this.sDrivetrain));

    // DO NOT USE THIS UNLESS YOU KNOW WHAT YOU ARE DOING!!!
    // driveToZero = new JoystickButton(stick, 2);
    // driveToZero.onTrue(new DriveToPosition(sDrivetrain, new Pose2d(0, 0, new Rotation2d())));

    travelatorForward = new JoystickButton(stick, 8);
    travelatorForward.whileTrue(sTravelator.moveTravelatorForward());

    travelatorBackward = new JoystickButton(stick, 9);
    travelatorBackward.whileTrue(sTravelator.moveTravelatorBackward());

    extendParkingBrake = new JoystickButton(stick, 3);
    extendParkingBrake.onTrue(sDrivetrain.extendParkingBrake());

    retractParkingBrake = new JoystickButton(stick, 4);
    retractParkingBrake.onTrue(sDrivetrain.retractParkingBrake());
    
    // travelatorInBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton2);
    // travelatorInBtn.whileHeld(new RunCommand(() -> sTravelator.Moveforward()));
    // travelatorInBtn.whenReleased(new InstantCommand(() -> sTravelator.Stop()));

    // travelatorOutBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton2);
    // travelatorOutBtn.whileHeld(new RunCommand(() -> sTravelator.MoveBackward()));
    // travelatorOutBtn.whenReleased(new InstantCommand(() -> sTravelator.Stop()));

    arm1Btn = new JoystickButton(stick, ctrlConstants.kJoystickButton2);
    arm1Btn.whileHeld(new InstantCommand(() -> sArm.arm1In()));
    arm1Btn.whenReleased(new InstantCommand(() -> sArm.arm1Stop()));

    arm1OutBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton3);
    arm1OutBtn.whileHeld(new InstantCommand(() -> sArm.arm1Out()));
    arm1OutBtn.whenReleased(new InstantCommand(() -> sArm.arm1Stop()));

    arm2Btn = new JoystickButton(stick, ctrlConstants.kJoystickButton4);
    arm2Btn.whileHeld(new InstantCommand(() -> sArm.arm2In()));
    arm2Btn.whenReleased(new InstantCommand(() -> sArm.arm2Stop()));

    arm2OutBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton6);
    arm2OutBtn.whileHeld(new InstantCommand(() -> sArm.arm2Out()));
    arm2OutBtn.whenReleased(new InstantCommand(() -> sArm.arm2Stop()));

    armGrabBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton1);
    armGrabBtn.whenPressed(new InstantCommand(() -> sArm.Grab()));

    // autoVisionBtn = new JoystickButton(controller, ctrlConstants.kXboxRightJoystickButton);
    // autoVisionBtn.toggleWhenPressed(new DriveVision(controller, sDrivetrain, sLimeLight));

    // gearBtn = new JoystickButton(controller, ctrlConstants.kXboxLeftJoystickButton);
    // gearBtn.whenPressed(new InstantCommand(() -> sDrivetrain.Gear()));
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

  /**
   * Gets the {@link Drivetrain} subsystem.
   *
   * @return {@link Drivetrain}
   */
  public Drivetrain getDrivetrain() {
    return this.sDrivetrain;
  }

  /**
   * Gets the {@link Arm} subsystem.
   *
   * @return {@link Arm}
   */
  public Arm getArm() {
    return this.sArm;
  }

  /**
   * Gets the {@link Travelator} subsystem.
   *
   * @return {@link Travelator}
   */
  public Travelator getTravelator() {
    return this.sTravelator;
  }

  /**
   * Gets the {@link LimeLight} subsystem.
   *
   * @return {@link LimeLight}
   */
  public LimeLight getLimeLight() {
    return this.sLimeLight;
  }

  /**
   * Gets the {@link Joystick}
   *
   * @return {@link Joystick}
   */
  public Joystick getStick() {
    return this.stick;
  }

  /**
   * Gets the {@link CommandXboxController}.
   *
   * @return {@link CommandXboxController}
   */
  public CommandXboxController getController() {
    return controller;
  }
}
