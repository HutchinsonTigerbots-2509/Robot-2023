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
import frc.robot.commands.Drive;
import frc.robot.commands.DriveApril;
import frc.robot.commands.DriveTele;
import frc.robot.commands.DriveVision;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.LimeLight;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.Constants.ctrlConstants;
import frc.robot.Constants.opConstants;
import frc.robot.Constants.camConstants;

import java.util.List;
import java.util.ResourceBundle.Control;

import org.opencv.photo.Photo;

import edu.wpi.first.wpilibj.SPI;

//import frc.robot.AutoCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static AprilTagFieldLayout aprilTagField = new AprilTagFieldLayout(
  List.of(
    new AprilTag(5, new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(147.19), Units.inchesToMeters(18.22), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0)))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(0))))
  ), camConstants.kFieldLength, camConstants.kFieldWidth);

  
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
  private PhotonVision sPhotonVision = new PhotonVision();

  // TODO Add in vision subsystems

  // ***** Joysticks ***** //
  private Joystick stick = new Joystick(Constants.kCoopStickID);
  private Joystick controller = new Joystick(Constants.kOpStickID);

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
  private JoystickButton secondAutoBtn;




                                                              // ***** WHERE YOU SET WHICH AUTO ***** //


  private AutoCommands mAutoCommands2 = AutoCommands.LEFT2;






  private Command Left = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot()), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1.25), //Drives Forward Stops With Command
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(6), //Run Conveyor Into Shooter Stops With Command
    new Drive(sDrivetrain, .3).withTimeout(.5), //Drives Forward Stops With Command
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()) //Stops Shooter
  );

  private Command Middle = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot()), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1.5), //Drives Forward Stops With Command
    new Drive(sDrivetrain, 0, 0, -.3).withTimeout(.10), //Drives Turnings Stops With Command
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(6), //Run Conveyor Into Shooter Stops With Command
    new Drive(sDrivetrain, .3).withTimeout(.5), //Drives Forward Stops With Command
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()) //Stops Shooter
  );

  private Command Right = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot(.6)), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1), //Drives Forward Stops With Command
    new Drive(sDrivetrain, 0).withTimeout(1),
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(1), //Run Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Pulls Up Intake
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.Shoot(.55)), //Slows Shooter
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(2), //Runs Conveyor Into Shooter Stops With 
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()), //Stops Shooter
    new Drive(sDrivetrain, 0, 0, .3).withTimeout(1), //Drives Turning Stops With Command
    new Drive(sDrivetrain, 0, -.5, 0).withTimeout(.5) //Drives Strafing Stops With Command
  );

  private Command Potato = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot()), //Starts Shooter
    new Drive(sDrivetrain, .3).withTimeout(1.5), //Drives Forward Stops With Command
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(1), //Run Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new InstantCommand(() -> sShooter.ShootStop()), //Stops Shooter
    new Drive(sDrivetrain, -.6).withTimeout(1) //Drives Backward Stops With Command
  );

  private Command Double = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot(.6)), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1), //Drives Forward Stops With Command
    new Drive(sDrivetrain, 0).withTimeout(1),
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(1), //Run Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Pulls Up Intake
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.Shoot(.55)), //Slows Shooter
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(2), //Runs Conveyor Into Shooter Stops With 
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new Drive(sDrivetrain, 0, 0, .4).withTimeout(.90), //Drives Turnings Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Puts Down Intake
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Turns On Intake
    new Drive(sDrivetrain, .3).withTimeout(2.7), //Drives Forward Stops With Command
    new Drive(sDrivetrain, 0, 0, -.3).withTimeout(.85), 
    new InstantCommand(() -> sShooter.Shoot()),
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(3), //Runs Conveyor Into Shooter Stops With
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor 
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()) //Stops Shooter
  );

  private Command MiddleFar = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot()), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.8)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1), //Drives Forward Stops With Command
    new Drive(sDrivetrain, 0, 0, -.3).withTimeout(.30), //Drives Turnings Stops With Command
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(2), //Run Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Pulls Up Intake
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.Shoot(.55)), //Slows Shooter
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(1), //Runs Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new Drive(sDrivetrain, 0, 0, .2).withTimeout(.45), //Drives Turnings Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Puts Down Intake
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Turns On Intake
    new Drive(sDrivetrain, .6).withTimeout(1.5), //Drives Forward Stops With Command
    new Drive(sDrivetrain, .2, .5, -.1).withTimeout(.8), //Drives Forward Stops With Command
    new InstantCommand(() -> sShooter.Shoot(.85)), //Starts Shooter
    new Drive(sDrivetrain, .3).withTimeout(.6),
    new Drive(sDrivetrain, -.3).withTimeout(.6),
    new AutoDriveVision(0, 0, sDrivetrain, sLimeLight).withTimeout(1),
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(5), //Runs Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()) //Stops Shooter
  );

  private Command DoubleFar = new SequentialCommandGroup(
    new InstantCommand(() -> sShooter.Shoot()), //Starts Shooter
    new InstantCommand(() -> sIntake.IntakeSetAuto()), //Puts Intake Down
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Starts The Intake
    new Drive(sDrivetrain, .3).withTimeout(1), //Drives Forward Stops With Command
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(2), //Run Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Pulls Up Intake
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)).withTimeout(2), //Runs Conveyor Into Shooter Stops With Command
    new InstantCommand(() -> sIntake.ConveyorStop()), //Stops Conveyor
    new Drive(sDrivetrain, 0, 0, .4).withTimeout(1), //Drives Turnings Stops With Command
    new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()), //Puts Down Intake
    new InstantCommand(() -> sIntake.IntakeIn(.5)), //Turns On Intake
    new Drive(sDrivetrain, .3).withTimeout(2), //Drives Forward Stops With Command
    new InstantCommand(() -> sIntake.IntakeOff()), //Turns Off Intake
    new InstantCommand(() -> sShooter.ShootStop()) //Stops Shooter
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    sDrivetrain.setDefaultCommand(new DriveTele(controller, sDrivetrain));

    AutoSelect.setDefaultOption("Right3", Double);
    AutoSelect.addOption("Middle2", Middle);
    AutoSelect.addOption("Right2", Right);
    AutoSelect.addOption("Left2", Left);
    AutoSelect.addOption("Middle4", MiddleFar);
    AutoSelect.addOption("Potato", Potato);
    SmartDashboard.putData(AutoSelect);
    

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

    conveyorBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton2);
    conveyorBtn.whileHeld(new RunCommand(() -> sIntake.ConveyorIn(opConstants.kMaxConveyorSpeed)));
    conveyorBtn.whenReleased(new InstantCommand(() -> sIntake.ConveyorStop()));

    conveyorOutBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton3);
    conveyorOutBtn.whileHeld(new RunCommand(() -> sIntake.ConveyorOut(opConstants.kMaxConveyorSpeed)));
    conveyorOutBtn.whenReleased(new InstantCommand(() -> sIntake.ConveyorStop()));

    // autoVisionBtn = new JoystickButton(controller, ctrlConstants.kXboxRightJoystickButton);
    // autoVisionBtn.toggleWhenPressed(new DriveVision(controller, sDrivetrain, sLimeLight));
    
    secondAutoBtn = new JoystickButton(controller, ctrlConstants.kXboxRightJoystickButton);
    secondAutoBtn.toggleWhenPressed(new DriveApril(controller, sDrivetrain, sPhotonVision));

    intakeBtn = new JoystickButton(controller, ctrlConstants.kXboxLeftBumper);
    intakeBtn.whileHeld(new RunCommand(() -> sIntake.IntakeIn()));
    intakeBtn.whenReleased(new RunCommand(() -> sIntake.IntakeOff()));
    intakeBtn.whenReleased(new InstantCommand(() -> sIntake.ConveyorStop()));

    intakeOutBtn = new JoystickButton(controller, ctrlConstants.kXboxButtonX);
    intakeOutBtn.whileHeld(new RunCommand(() -> sIntake.IntakeOut(-opConstants.kIntakeSpeed)));
    intakeOutBtn.whenReleased(new RunCommand(() -> sIntake.IntakeOff()));

    shooterBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton1);
    shooterBtn.whileHeld(new RunCommand(() -> sShooter.Shoot()));
    shooterBtn.whenReleased(new InstantCommand(() -> sShooter.ShootStall()));

    intakeToggleBtn = new JoystickButton(controller, ctrlConstants.kXboxRightBumper);
    intakeToggleBtn.whenPressed(new InstantCommand(() -> sIntake.ToggleIntakeSolenoid()));

    shootSpeedBtn1 = new JoystickButton(stick, ctrlConstants.kJoystickButton7);
    shootSpeedBtn1.whenPressed(new InstantCommand(() -> sShooter.ShootSpeed1()));

    shootSpeedBtn2 = new JoystickButton(stick, ctrlConstants.kJoystickButton8);
    shootSpeedBtn2.whenPressed(new InstantCommand(() -> sShooter.ShootSpeed2()));

    shootSpeedBtn3 = new JoystickButton(stick, ctrlConstants.kJoystickButton9);
    shootSpeedBtn3.whenPressed(new InstantCommand(() -> sShooter.ShootSpeed3()));

    // gearUpBtn = new JoystickButton(controller, Constants.kXboxButtonY);
    // gearUpBtn.whenPressed(new InstantCommand(() -> sDrivetrain.GearUp()));

    // gearDownBtn = new JoystickButton(controller, Constants.kXboxButtonA);
    // gearDownBtn.whenPressed(new InstantCommand(() -> sDrivetrain.GearDown()));

    gearBtn = new JoystickButton(controller, ctrlConstants.kXboxLeftJoystickButton);
    gearBtn.whenPressed(new InstantCommand(() -> sDrivetrain.Gear()));

    climbUpBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton6);
    climbUpBtn.whenPressed(new InstantCommand(() -> sShooter.ShootStop()));
    climbUpBtn.whenPressed(new InstantCommand(() -> sClimb.ClimbUp()));
    climbUpBtn.whenReleased(new InstantCommand(() -> sClimb.ClimbStop()));

    climbDownBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton4);
    climbDownBtn.whenPressed(new InstantCommand(() -> sShooter.ShootStop()));
    climbDownBtn.whenPressed(new InstantCommand(() -> sClimb.ClimbDown()));
    climbDownBtn.whenReleased(new InstantCommand(() -> sClimb.ClimbStop()));

    climbToggleBtn = new JoystickButton(stick, ctrlConstants.kJoystickButton11);
    climbToggleBtn.whenPressed(new InstantCommand(() -> sShooter.ShootStop()));
    climbToggleBtn.whenPressed(new InstantCommand(() -> sClimb.ClimbSolenoidToggle()));

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
  public PhotonVision getPhotonVision() { return sPhotonVision;}

  public Joystick getStick() { return stick; }
  public Joystick getController() { return controller; }
  
}