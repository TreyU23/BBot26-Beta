// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.JumpIntake;
import frc.robot.Commands.ReverseFeeding;
import frc.robot.Commands.SmartShooter;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Turret;
import frc.robot.Utilities.AutoValidator;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriver);
  private final CommandXboxController m_operatorController = new CommandXboxController(DriverConstants.kOperator);

  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Intake m_intake = new Intake();
  private final Kicker m_kicker = new Kicker();
  private final Shooter m_shooter = new Shooter();
  private final Spindexer m_spindexer = new Spindexer();
  private final Turret m_turret = new Turret(() -> m_drivetrain.getState().Speeds.omegaRadiansPerSecond);

  private final Command m_fieldRelativeDrive = DriveCommands.driveRequest(m_drivetrain,
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX(), () -> m_driverController.getRightX(), () -> true);
  private final SmartShooter m_smartShooter = new SmartShooter(m_shooter, m_turret, m_drivetrain::getState,
      true);
  private final JumpIntake m_jumpIntake = new JumpIntake(m_intake);
  private final FeedShooter m_feedShooter = new FeedShooter(m_spindexer, m_kicker, m_turret::atSetpoint);
  private final ReverseFeeding m_reverseShooter = new ReverseFeeding(m_spindexer, m_kicker);

  private final SendableChooser<String> m_autoChooser;

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();

    m_autoChooser = AutoValidator.buildChooser("Auto Chooser");
  }

  private void configureDefaultCommands() {
    m_drivetrain.setDefaultCommand(m_fieldRelativeDrive);
  }

  private void configureBindings() {
    m_driverController.start().onTrue(DriveCommands.resetFieldOrientation(m_drivetrain));

    m_driverController.leftBumper()
        .whileTrue(m_intake.setIntake(IntakeConstants.kIntakePose, IntakeConstants.kIntakeVoltage)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    m_driverController.leftTrigger().whileTrue(m_smartShooter);

    m_driverController.rightTrigger()
        .whileTrue(m_feedShooter.onlyWhile(m_smartShooter::isScheduled))
        .onTrue(Commands.runOnce(() -> m_shooter.allowHood(true)))
        .onFalse(Commands.runOnce(() -> m_shooter.allowHood(false)));

    m_driverController.y().and(m_operatorController.y())
        .whileTrue(m_jumpIntake.withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    m_driverController.b().and(m_operatorController.b()).whileTrue(m_reverseShooter);

    m_operatorController.leftBumper()
        .whileTrue(m_intake.zeroIntakeCmd().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  public Command getAutonomousCommand() {
    return AutoValidator.getSelectedCommand();
  }

  public void configureNamedCommands() {
  }
}
