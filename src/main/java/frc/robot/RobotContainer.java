// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveVision;
import frc.robot.commands.OrientalDrive;
import frc.robot.commands.Travelator.TravelatorMoveToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.Constants.ctrlConstants;
import frc.robot.Constants.opConstants;

//import frc.robot.AutoCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ***** Select Auto ***** //
  SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // ***** Subsystems ***** //
  private Drivetrain sDrivetrain = new Drivetrain();
  private LimeLight sLimeLight = new LimeLight();
  private Arm sArm = new Arm();
  private Travelator sTravelator = new Travelator();

  // ***** Joysticks ***** //
  private Joystick coopStick = new Joystick(Constants.kCoopStickID);
  private Joystick opStick = new Joystick(Constants.kOpStickID);

  // ***** Joystick Buttons ***** //
  private Trigger armForwardBtn;
  private Trigger armBackwardBtn;
  private Trigger travelatorForwardBtn;
  private Trigger travelatorBackwardBtn;
  private Trigger travelatorBackPosBtn;
  private Trigger travelatorMiddlePosBtn;
  private Trigger travelatorFrontPosBtn;
  private Trigger armWristForwardBtn;
  private Trigger armWristBackwardBtn;
  private Trigger armKnuckleForwardBtn;
  private Trigger armKnuckleBackwardBtn;
  private Trigger extendParkingBrake;
  private Trigger retractParkingBrake;
  private Trigger grabBtn;
  private Trigger POVTriggerBtn;

  //private AutoCommands mAutoCommands2 = AutoCommands.LEFT2;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    sDrivetrain.setDefaultCommand(new OrientalDrive(opStick, sDrivetrain));

    // AutoSelect.setDefaultOption("Right3", Double);
    // AutoSelect.addOption("Middle2", Middle);
    // AutoSelect.addOption("Right2", Right);
    // AutoSelect.addOption("Left2", Left);
    // AutoSelect.addOption("Middle4", MiddleFar);
    // AutoSelect.addOption("Potato", Potato);
    SmartDashboard.putData(AutoSelect);
    
    // Configure the button bindings
    configureButtonBindings();

    // travelatorInBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton9);
    // travelatorInBtn.whileTrue(new RunCommand(() -> sTravelator.Moveforward()));
    // travelatorInBtn.onFalse(new InstantCommand(() -> sTravelator.Stop()));

    // travelatorOutBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton10);
    // travelatorOutBtn.whileTrue(new RunCommand(() -> sTravelator.MoveBackward()));
    // travelatorOutBtn.onFalse(new InstantCommand(() -> sTravelator.Stop()));

    armForwardBtn = new JoystickButton(coopStick, 7);
    armForwardBtn.whileTrue(sArm.cmdArmLiftForward());

    armBackwardBtn = new JoystickButton(coopStick,8);
    armBackwardBtn.whileTrue(sArm.cmdArmLiftBackward());

    armWristForwardBtn = new JoystickButton(coopStick, 5);
    armWristForwardBtn.whileTrue(sArm.cmdArmWristForward());

    armWristBackwardBtn = new JoystickButton(coopStick,3);
    armWristBackwardBtn.whileTrue(sArm.cmdArmWristBackward());

    armKnuckleForwardBtn = new JoystickButton(coopStick, 6);
    armKnuckleForwardBtn.whileTrue(sArm.cmdArmKnuckleIn());

    armKnuckleBackwardBtn = new JoystickButton(coopStick, 4);
    armKnuckleBackwardBtn.whileTrue(sArm.cmdArmKnuckleOut());

    extendParkingBrake = new JoystickButton(opStick, 3);
    extendParkingBrake.onTrue(sDrivetrain.extendParkingBrake());

    retractParkingBrake = new JoystickButton(opStick, 5);
    retractParkingBrake.onTrue(sDrivetrain.retractParkingBrake());

    grabBtn = new JoystickButton(coopStick, 1);
    grabBtn.onTrue(sArm.Grab());

    //Travelator Button
    travelatorForwardBtn = new JoystickButton(opStick, 7);
    travelatorForwardBtn.whileTrue(sTravelator.cmdMoveForward());

    travelatorBackwardBtn = new JoystickButton(opStick, 11);
    travelatorBackwardBtn.whileTrue(sTravelator.cmdMoveBackward());

    travelatorBackPosBtn = new JoystickButton(opStick, 12);
    travelatorBackPosBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorBack, sTravelator));

    travelatorMiddlePosBtn = new JoystickButton(opStick, 10);
    travelatorMiddlePosBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorMiddle, sTravelator));

    travelatorFrontPosBtn = new JoystickButton(opStick, 8);
    travelatorFrontPosBtn.onTrue(new TravelatorMoveToPosition(opConstants.kTravelatorFront, sTravelator));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoSelect.getSelected();
  }

  public Command getAutCommand(){
    return AutoSelect.getSelected();
  }
  
  // Getter Methods

  public Drivetrain getDrivetrain() { return sDrivetrain; }
  public Arm getArm() { return sArm; }
  public Travelator gTravelator() { return sTravelator; }
  public LimeLight getLimeLight() { return sLimeLight; }

  public Joystick getCoopStick() { return coopStick; }
  public Joystick getOpStick() { return opStick; }
  
}
