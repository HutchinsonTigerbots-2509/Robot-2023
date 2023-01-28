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
import frc.robot.commands.auto.Rightsingle;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  private Intake sIntake = new Intake();
  private Shooter sShooter = new Shooter();
  private Conveyor sConveyor = new Conveyor();
  private Climb sClimb = new Climb();
  private LimeLight sLimeLight = new LimeLight();

  // TODO Add in vision subsystems

  // ***** Joysticks ***** //
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  // private Joystick controller = new Joystick(Constants.kOpStickID);

  // ***** Joystick Buttons ***** //
  private Trigger turnToZero;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // AutoSelect.setDefaultOption("Right3", Rightsingle );
    // AutoSelect.addOption("Middle2", Middle);
    // AutoSelect.addOption("Right2", Right);
    // AutoSelect.addOption("Left2", Left);
    // AutoSelect.addOption("Middle4", MiddleFar);
    // AutoSelect.addOption("Potato", Potato);
    // SmartDashboard.putData(AutoSelect);
    
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
      /* switch(mAutoCommands2) {
        case RIGHT2:
          return Right;
        case MIDDLE2:
          return Middle;
        case LEFT2:
          return Left;
        case POTATO:
          return Potato;
        case RIGHT3:
          return Double;
        case MIDDLE4:
          return MiddleFar;
        default:
          return Right;
     } */
  }

  public Command getAutCommand(){
    return AutoSelect.getSelected();
  }
  
  // Getter Methods

  public Drivetrain getDrivetrain() { return sDrivetrain; }
  public Intake getIntake() { return sIntake; }
  public Shooter getShooter() { return sShooter; }
  public Conveyor getConveyor() { return sConveyor; }
  public Climb getClimb() { return sClimb; }
  public LimeLight getLimeLight() { return sLimeLight; }

  public Joystick getStick() { return stick; }
  // public Joystick getController() { return controller; }
  public Joystick getController() { return null;}
}
