// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDriveVision;
import frc.robot.commands.DriveTele;
import frc.robot.commands.DriveVision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Travelator;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.Constants.ctrlConstants;
import frc.robot.Constants.opConstants;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.SPI;

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


  // ***** Nav X ***** //
  //AHRS NavX;
  //float DisplacementX = NavX.getDisplacementX();
  //float DisplacementY = NavX.getDisplacementY();
  
  // ***** Subsystems ***** //
  private Drivetrain sDrivetrain = new Drivetrain();
  private LimeLight sLimeLight = new LimeLight();
  private Arm sArm = new Arm();
  private Travelator sTravelator = new Travelator();

  // ***** Joysticks ***** //
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  // private Joystick controller = new Joystick(Constants.kOpStickID);

  // ***** Joystick Buttons ***** //
  private JoystickButton conveyorBtn;
  private JoystickButton conveyorOutBtn;
  private JoystickButton shooterBtn;
  private JoystickButton intakeToggleBtn;
  private JoystickButton gearUpBtn;
  private JoystickButton gearDownBtn;
  private JoystickButton intakeBtn;
  private JoystickButton intakeOutBtn;
  private JoystickButton climbUpBtn;
  private JoystickButton climbDownBtn;
  private JoystickButton climbToggleBtn;
  private JoystickButton shootSpeedBtn1;
  private JoystickButton shootSpeedBtn2;
  private JoystickButton shootSpeedBtn3;
  private JoystickButton autoVisionBtn;
  private JoystickButton gearBtn;




                                                              // ***** WHERE YOU SET WHICH AUTO ***** //


  // private AutoCommands mAutoCommands2 = AutoCommands.LEFT2;











  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    sDrivetrain.setDefaultCommand(new DriveTele(stick, sDrivetrain));

 

    // try 
    //   {NavX = new AHRS(SPI.Port.kMXP);}
    // catch (RuntimeException ex ) 
    //   {DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);}

    // autoSelect.setDefaultOption("RED_RIGHT", redRight);
    // autoSelect.addOption("RED_MIDDLE", redMiddle);
    // autoSelect.addOption("RED_LEFT", redLeft);
    // autoSelect.addOption("BLUE_RIGHT", blueRight);
    // autoSelect.addOption("BLUE_MIDDLE", blueMiddle);
    // autoSelect.addOption("BLUE_LEFt", blueLeft);

    /*public Command getAutoCommand(){
      return autoSelect.getSelected();
    }*/

    // Configure the button bindings
    configureButtonBindings();


    autoVisionBtn = new JoystickButton(stick, ctrlConstants.kXboxRightJoystickButton);
    autoVisionBtn.toggleWhenPressed(new DriveVision(stick, sDrivetrain, sLimeLight));

    

    // gearUpBtn = new JoystickButton(controller, Constants.kXboxButtonY);
    // gearUpBtn.whenPressed(new InstantCommand(() -> sDrivetrain.GearUp()));

    // gearDownBtn = new JoystickButton(controller, Constants.kXboxButtonA);
    // gearDownBtn.whenPressed(new InstantCommand(() -> sDrivetrain.GearDown()));

    // gearBtn = new JoystickButton(stick, ctrlConstants.kXboxLeftJoystickButton);
    // gearBtn.whenPressed(new InstantCommand(() -> sDrivetrain.Gear()));



  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /*public Command getAutoChoice()
  {
    return autoSelect.getSelected;
  }*/

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

  public Drivetrain getDrivetrain() { return sDrivetrain; }
  public LimeLight getLimeLight() { return sLimeLight; }


  public Joystick getStick() {
    return stick;
  }
  // public Joystick getController() { return controller; }
  public Joystick getController() {
    return null;
  }
}
