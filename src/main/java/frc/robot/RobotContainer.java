 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.*;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.subsystems.*;
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

  /** Autos **/
  private Potato cmdPotato = new Potato();
  private LeftSingle cmdLeftSing = new LeftSingle();
  private LeftSingleCharger cmdLeftCharge = new LeftSingleCharger();
  private MiddleSingleCharger cmdMidCharge = new MiddleSingleCharger();
  private RightSingle cmdRightSing = new RightSingle();
  private RightSingleCharge cmdRightCharge = new RightSingleCharge();


  /** Nav-X **/
  //AHRS NavX;
  //float DisplacementX = NavX.getDisplacementX();
  //float DisplacementY = NavX.getDisplacementY();
  
  /** Subsystems **/
  private Drivetrain sDrivetrain = new Drivetrain();
  private LimeLight sLimeLight = new LimeLight();

  // TODO Add in vision subsystems

  // ***** Joysticks ***** //
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  // private Joystick controller = new Joystick(Constants.kOpStickID);

  // ***** Joystick Buttons ***** //
  private Trigger turnToZero;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutoSelect.setDefaultOption("Potato", cmdPotato);
    AutoSelect.addOption("Left Single", cmdLeftSing);
    AutoSelect.addOption("Left Charge", cmdLeftCharge);
    AutoSelect.addOption("Mid Charge", cmdMidCharge);
    AutoSelect.addOption("Right Single", cmdRightCharge);
    AutoSelect.addOption("Right Charge", cmdRightCharge);
    
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

  }

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
  public LimeLight getLimeLight() { return sLimeLight; }

  public Joystick getStick() { return stick; }
  // public Joystick getController() { return controller; }
  public Joystick getController() { return null;}
}
