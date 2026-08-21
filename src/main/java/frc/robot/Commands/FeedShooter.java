package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Spindexer;

public class FeedShooter extends Command {
    private final Spindexer m_spindexer;
    private final Kicker m_kicker;

    private final BooleanSupplier m_finishedMove;
    private final BooleanSupplier m_dontFeedShooters;

    public FeedShooter(Spindexer spindexer, Kicker kicker, BooleanSupplier turretAtTarget, BooleanSupplier dontFeed) {
        m_spindexer = spindexer;
        m_kicker = kicker;
        m_finishedMove = turretAtTarget;
        m_dontFeedShooters = dontFeed;
        addRequirements(m_spindexer, m_kicker);
    }

    @Override
    public void initialize() {
        DriveCommands.updateMAX(DriverConstants.SLOWED_SPEED, DriverConstants.SLOWED_ANGULAR_RATE);
        SmartDashboard.putBoolean("isSlowed", true);
    }

    @Override
    public void execute() {
        if (m_finishedMove.getAsBoolean() && !m_dontFeedShooters.getAsBoolean()) {
            m_spindexer.setVoltage(SpindexerConstants.kRunVoltage);
            m_kicker.setVoltage(KickerConstants.kRunVoltage);
        } else {
            m_spindexer.stop();
            m_kicker.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_kicker.stop();
        m_spindexer.stop();

        DriveCommands.updateMAX(DriverConstants.MAX_SPEED, DriverConstants.MAX_ANGULAR_RATE);
        SmartDashboard.putBoolean("isSlowed", false);
    }
}
