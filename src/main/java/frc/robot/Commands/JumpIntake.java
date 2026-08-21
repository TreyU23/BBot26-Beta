package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class JumpIntake extends Command {
    private final Intake m_intake;
    private final Timer m_time = new Timer();

    private final double m_upSetpoint = -0.25;
    private final double m_minDownSetpoint = -0.20;
    private double m_downSetpoint = -0.025;

    public JumpIntake(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setVoltage(2);
        m_downSetpoint = -0.025;

        m_time.reset();
        m_time.start();
    }

    @Override
    public void execute() {
        double currentTime = m_time.get();

        if (currentTime < 0.3) {
            m_intake.setPosition(m_downSetpoint);
        } else if (currentTime <= 0.6 && currentTime >= 0.3) {
            m_intake.setPosition(m_upSetpoint);
        } else if (currentTime >= 0.6) {
            if (m_downSetpoint > m_minDownSetpoint + 0.0125) {
                m_downSetpoint -= 0.025;
            }
            m_time.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_time.stop();
        m_intake.setPosition(0.0);
        m_intake.stopRoller();
    }
}
