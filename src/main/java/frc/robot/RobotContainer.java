// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Utilities.AutoValidator;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriver);
  private final CommandXboxController m_operatorController = new CommandXboxController(DriverConstants.kOperator);

  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  private final SendableChooser<String> m_autoChooser;

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();

    m_autoChooser = AutoValidator.buildChooser("Auto Chooser");
  }

  private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(DriveCommands.driveRequest(m_drivetrain, () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(), () -> m_driverController.getRightX(), () -> true));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return AutoValidator.getSelectedCommand();
  }
}
