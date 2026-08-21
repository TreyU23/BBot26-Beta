package frc.robot.Commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class TurretToGoal extends Command {
    private final Turret m_turret;
    private final String m_name;

    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<Translation2d> m_goalSupplier;

    public TurretToGoal(Turret turret, Supplier<Pose2d> 
        robotPoseSupplier, Supplier<Translation2d> goalSupplier, String name) {

        m_turret = turret;
        m_robotPoseSupplier = robotPoseSupplier;
        m_goalSupplier = goalSupplier;
        m_name = name;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Translation2d turretFR = m_turret.getTranslation().rotateBy(m_robotPoseSupplier.get().getRotation());
        Translation2d turretToGoal = m_goalSupplier.get().minus(m_robotPoseSupplier.get().getTranslation().plus(turretFR));
        Rotation2d alpha = turretToGoal.getAngle();
        m_turret.setPosition(alpha.getRadians()-m_robotPoseSupplier.get().getRotation().getRadians());

        SmartDashboard.putNumber(m_name + " Distance to Goal", turretToGoal.getNorm());
    }


    @Override
    public void end(boolean interrupted) {
        //m_turret.setPosition(0.0);
    }
}
