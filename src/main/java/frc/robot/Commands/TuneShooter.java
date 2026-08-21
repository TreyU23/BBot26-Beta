package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class TuneShooter extends Command {
    private final Shooter m_shooter;

    public TuneShooter(Shooter shooter){
        m_shooter = shooter;
        SmartDashboard.putNumber("Shooter Velocity Tuning", 0);
        SmartDashboard.putNumber("Shooter Position Tuning", 1.0);
        addRequirements(m_shooter);

    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_shooter.allowHood(true);
    }
    @Override
    public void execute() {
        m_shooter.setVelocity(SmartDashboard.getNumber(m_shooter.getName() + " Velocity Tuning", 0));
        m_shooter.setPosition(SmartDashboard.getNumber(m_shooter.getName() + " Position Tuning", 0));
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.allowHood(false);
        m_shooter.setPosition(1.0);
        m_shooter.stopRoller();
        // TODO Auto-generated method stub
        super.end(interrupted);
    } 
}
