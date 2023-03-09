// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Claw.Claw;
import frc.robot.commands.Swerve.Drive;
import frc.robot.commands.test.testMotorCommand;
import frc.robot.constants.Ports;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.CompressorModule;
import frc.robot.subsystems.modules.PDH;
import frc.robot.test.armTest;
import frc.robot.test.clawTest;
import frc.robot.test.intakeTest;
import frc.robot.test.testWhatever;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Subsystems */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();
  private final PDH powerDistributionHub = new PDH();
  private final testWhatever testers = new testWhatever();

  private final CompressorModule 
  compressor = new CompressorModule();

  //private final armTest arm = new armTest();
  //private final intakeTest intake = new intakeTest();

  /* Commands */

  // Replace with CommandPS4Controller or CommandJoystick if needed
  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);

  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  
  private final boolean power = DRIVER.getAButton();

  private final testMotorCommand testmotor = new testMotorCommand(testers, power);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  private final Trigger robotCentric = new JoystickButton(DRIVER, XboxController.Button.kY.value);

  private final Trigger slowSpeed = new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);

  // Additional buttons
  private final Trigger highGrid = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

  private final Trigger tester = new JoystickButton(DRIVER, XboxController.Button.kB.value);
  
  private final Trigger tester2 = new JoystickButton(DRIVER, XboxController.Button.kX.value);

  //Controller two
  private final Trigger Abutton2 = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger Bbutton2 = new JoystickButton(OPERATOR, XboxController.Button.kB.value);
  private final Trigger Xbutton2 = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger Ybutton2 = new JoystickButton(OPERATOR, XboxController.Button.kY.value);

  private final Trigger RightBumper2 = new JoystickButton(OPERATOR, XboxController.Button.kRightBumper.value);
  private final Trigger leftBumper2 = new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  private final Trigger startButton2 = new JoystickButton(OPERATOR, XboxController.Button.kStart.value);
  private final Trigger BackButton2 = new JoystickButton(OPERATOR, XboxController.Button.kBack.value);

  private final Trigger LeftStickButton2 = new JoystickButton(OPERATOR, XboxController.Button.kLeftStick.value);
  private final Trigger RightStickButton2 = new JoystickButton(OPERATOR, XboxController.Button.kRightStick.value);


  // TODO May need to switch the object for each button to JoystickButton
  // Create SmartDashboard chooser for autonomous routines
  private static SendableChooser<Command> Autons = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SwerveDrive.setDefaultCommand(
        new Drive(
            SwerveDrive,
            () -> -DRIVER.getRawAxis(translationAxis),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean()));

    // Configure the trigger bindings, defaults, Autons
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    compressor.enableAnalog(0, 120);
    //compressor.enableDigital();//FIXME Try later
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // DRVIER.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));// A value for the Xbox Controller
    
    tester.whileTrue(new InstantCommand(() -> testers.upGoArm()));
    tester.whileFalse(new InstantCommand(() -> testers.stopArm()));

    tester2.whileTrue(new InstantCommand(() -> testers.downGoArm()));
    tester2.whileFalse(new InstantCommand(() -> testers.stopArm()));

    //Compressor 
    Abutton2.onTrue(new InstantCommand(()-> testers.upGoClaw()));
    Abutton2.onFalse(new InstantCommand(()-> testers.stopClaw()));

    Bbutton2.onTrue(new InstantCommand(()-> testers.downGoClaw()));
    Bbutton2.onFalse(new InstantCommand(()-> testers.stopClaw()));

    //solenoids
    Xbutton2.onTrue(new InstantCommand(()-> testers.openClaw()));
    Ybutton2.onTrue(new InstantCommand(()-> testers.closeClaw()));

    leftBumper2.onTrue(new InstantCommand(()-> testers.upGoWrist()));
    leftBumper2.onFalse(new InstantCommand(()-> testers.stopWrist()));

    RightBumper2.onTrue(new InstantCommand(()-> testers.downGoWrist()));
    RightBumper2.onFalse(new InstantCommand(()-> testers.stopWrist()));


    //extender and wrist needs another button to go back
    startButton2.onTrue(new InstantCommand(()-> testers.upExtender()));
    startButton2.onFalse(new InstantCommand(()-> testers.stopExtender()));

    BackButton2.onTrue(new InstantCommand(()-> testers.downExtender()));
    BackButton2.onFalse(new InstantCommand(()-> testers.stopExtender()));

    LeftStickButton2.whileFalse((new InstantCommand(()-> compressor.disableCompressor())));
    RightStickButton2.onTrue(new InstantCommand(()-> compressor.enableAnalog(0, 120)));


    // System.out.print("Swervy");

    // slowSpeed.onTrue(new InstantCommand(() -> SwerveDrive.slowSpeed()));

    // robotCentric.onTrue(new InstantCommand(() -> SwerveDrive.robotCentric()));

  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    // autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
    SmartDashboard.putData("Autonomous: ", Autons);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autons.getSelected();
  }
}
