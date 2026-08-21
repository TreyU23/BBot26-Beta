package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Spindexer;

public class ReverseFeeding extends Command {
    private final Spindexer m_spindexer;
    private final Kicker m_kicker;

    public ReverseFeeding(Spindexer spindexer, Kicker kicker) {
        m_spindexer = spindexer;
        m_kicker = kicker;
    }

    @Override
    public void initialize() {
        m_spindexer.setVoltage(-2.0);
        m_kicker.setVoltage(2.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
        m_kicker.stop();
    }
}
