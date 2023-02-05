// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTele;
import frc.robot.commands.DriveVision;
import frc.robot.commands.OrientalDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.Constants.ctrlConstants;

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
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  private Joystick controller = new Joystick(Constants.kOpStickID);

  // ***** Joystick Buttons ***** //
  private JoystickButton arm1Btn;
  private JoystickButton arm1OutBtn;
  private JoystickButton arm2Btn;
  private JoystickButton arm2OutBtn;
  private JoystickButton travelatorOutBtn;
  private JoystickButton travelatorInBtn;
  private JoystickButton autoVisionBtn;
  private JoystickButton gearBtn;



  //private AutoCommands mAutoCommands2 = AutoCommands.LEFT2;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    sDrivetrain.setDefaultCommand(new DriveTele(controller, sDrivetrain));

    // AutoSelect.setDefaultOption("Right3", Double);
    // AutoSelect.addOption("Middle2", Middle);
    // AutoSelect.addOption("Right2", Right);
    // AutoSelect.addOption("Left2", Left);
    // AutoSelect.addOption("Middle4", MiddleFar);
    // AutoSelect.addOption("Potato", Potato);
    SmartDashboard.putData(AutoSelect);
    
    // Configure the button bindings
    configureButtonBindings();

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

    // autoVisionBtn = new JoystickButton(controller, ctrlConstants.kXboxRightJoystickButton);
    // autoVisionBtn.toggleWhenPressed(new DriveVision(controller, sDrivetrain, sLimeLight));

    // gearBtn = new JoystickButton(controller, ctrlConstants.kXboxLeftJoystickButton);
    // gearBtn.whenPressed(new InstantCommand(() -> sDrivetrain.Gear()));
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

  public Joystick getStick() { return stick; }
  public Joystick getController() { return controller; }
  
}
