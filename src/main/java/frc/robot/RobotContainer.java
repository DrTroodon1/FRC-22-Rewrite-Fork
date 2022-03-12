// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.OneCargoAuto;
import frc.robot.commands.operatorcommands.TeleClimbDown;
import frc.robot.commands.operatorcommands.TeleClimbUp;
import frc.robot.commands.operatorcommands.TeleIntake;
import frc.robot.commands.operatorcommands.TeleIntakeToggle;
import frc.robot.commands.operatorcommands.TeleShoot;
import frc.robot.commands.operatorcommands.TeleShootStop;
import frc.robot.commands.operatorcommands.TeleUptake;
import frc.robot.oi.UserControls;
import frc.robot.oi.XboxUserControls;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.drivetrain.DriveBase;
import frc.robot.subsystems.drivetrain.DriveIOFalcon;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelIONonVariable;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.transversal.Transversal;
import frc.robot.subsystems.transversal.TransversalIOSparkMax;
import frc.robot.subsystems.uptake.Uptake;
import frc.robot.subsystems.uptake.UptakeIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase drive = new DriveBase(new DriveIOFalcon());
  private final Climber climber = new Climber(
      new ClimberIOFalcon(ClimberIOFalcon.leftClimberPort),
      new ClimberIOFalcon(ClimberIOFalcon.rightClimberPort));
  private final Intake intake = new Intake(new IntakeIOTalonSRX());
  private final FlyWheel flyWheel = new FlyWheel(new FlyWheelIONonVariable());
  private final Transversal transversal = new Transversal(new TransversalIOSparkMax());
  private final Uptake uptake = new Uptake(new UptakeIOSparkMax());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    UserControls controls = new XboxUserControls(0, 1);

    // Define commands here
    DriveWithJoystick defaultDriveCommand = new DriveWithJoystick(
        drive,
        () -> Constants.mode,
        () -> controls.getLeftDriveX(),
        () -> controls.getLeftDriveY(),
        () -> controls.getRightDriveX(),
        () -> controls.getRightDriveY());

    TeleShoot shootCommand = new TeleShoot(flyWheel,
        transversal,
        uptake,
        () -> controls.defaultVoltage());

    TeleShootStop shootStopCommand = new TeleShootStop(flyWheel,
        transversal,
        uptake,
        () -> controls.zeroValue());

    TeleIntake defaultIntakeCommand = new TeleIntake(intake,
        () -> controls.getIntakeSpeed());

    TeleUptake defaultUptakeCommand = new TeleUptake(uptake,
        () -> controls.getUptakeSpeed());

    TeleClimbUp climberUpCommand = new TeleClimbUp(climber);
    TeleClimbDown climberDownCommand = new TeleClimbDown(climber);
    TeleIntakeToggle intakeToggleCommand = new TeleIntakeToggle(intake);

    // Define default commands here
    drive.setDefaultCommand(defaultDriveCommand);
    intake.setDefaultCommand(defaultIntakeCommand);
    uptake.setDefaultCommand(defaultUptakeCommand);

    // Define button / command bindings here
    controls.getClimbUp().whileActiveOnce(climberUpCommand);
    controls.getClimbDown().whileActiveOnce(climberDownCommand);
    controls.getShootButton().whenInactive(shootStopCommand);
    controls.getShootButton().whileActiveOnce(shootCommand);
    controls.getIntakeExtendButton().whenActive(intakeToggleCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new OneCargoAuto(drive);
  }
}
