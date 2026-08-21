package frc.robot.Commands;

import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;

public class SmartShooter extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Supplier<SwerveDriveState> m_robotStateSupplier;
    private boolean m_feedShot = false;
    private boolean m_dontFeed = false;
    private final boolean m_moveAndShot;

    /**
     * Forward-prediction (lookahead) time for latency compensation, in seconds.
     *
     * This accounts for the delay between sampling robot state and the aiming
     * command actually taking effect -- roughly one main loop period plus a little
     * actuator/command lag. With a CANivore (CAN FD) bus and Phoenix 6
     * latency-compensated signals, sensor/odometry latency is small (the odometry
     * thread runs up to 250 Hz on CANivore), so the loop period dominates.
     *
     * Start at one TimedRobot loop (0.020 s). Tune empirically: increase if the
     * robot consistently trails its aim while translating, decrease if it leads.
     * Keep it well under your ball flight time -- the in-flight motion is handled
     * separately by compensateForMotion(). Consider moving this to ShooterConstants.
     */
    private static final double kLookaheadSeconds = 0.030;

    public SmartShooter(Shooter shooter, Turret turret,
            Supplier<SwerveDriveState> robotStateSupplier,
            boolean moveAndShoot) {
        m_robotStateSupplier = robotStateSupplier;
        this.setName(shooter.getName() + " AutoAlign");
        m_shooter = shooter;
        m_turret = turret;
        m_moveAndShot = moveAndShoot;
        addRequirements(m_shooter, m_turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        boolean isBlue = alliance == DriverStation.Alliance.Blue;

        // One snapshot so pose and speeds come from the same instant.
        SwerveDriveState state = m_robotStateSupplier.get();
        Pose2d currentPose = state.Pose;
        ChassisSpeeds robotRelSpeeds = state.Speeds;

        // Latency compensation, done correctly: predict where the robot will be when
        // this aiming command actually takes effect. exp() integrates a ROBOT-RELATIVE
        // twist (constant-curvature) in the pose's local frame and returns a NEW pose
        // (Pose2d is immutable), so we capture the result in robotPose and derive every
        // downstream quantity -- turret pose, field-relative speeds, turret setpoint --
        // from it. That consistency is the whole point; predicting the pose but aiming
        // from the old one would be worse than not predicting at all.
        Twist2d lookaheadTwist = new Twist2d(
                robotRelSpeeds.vxMetersPerSecond * kLookaheadSeconds,
                robotRelSpeeds.vyMetersPerSecond * kLookaheadSeconds,
                robotRelSpeeds.omegaRadiansPerSecond * kLookaheadSeconds);
        Pose2d robotPose = currentPose.exp(lookaheadTwist);

        Translation2d goalLocation;

        m_dontFeed = false;

        if (isBlue) {
            goalLocation = GoalConstants.Blue.kHub;
            SignalLogger.writeString("SmartShooter Goal", "Blue Hub");
        } else {
            goalLocation = GoalConstants.Red.kHub;
            SignalLogger.writeString("SmartShooter Goal", "Red Hub");
        }

        if (robotPose.getX() >= goalLocation.getX() && isBlue) {
            m_feedShot = true;
        } else if (robotPose.getX() <= goalLocation.getX() && !isBlue) {
            m_feedShot = true;
        } else {
            m_feedShot = false;
        }

        SmartDashboard.putNumber("Base Goal Location X", goalLocation.getX());
        SmartDashboard.putNumber("Base Goal Location Y", goalLocation.getY());

        Translation2d fieldRelTurretPose = robotPose.getTranslation()
                .plus(m_turret.getTranslation().rotateBy(robotPose.getRotation()));

        if (m_feedShot) {
            if (isBlue) {
                if (fieldRelTurretPose.getY() > goalLocation.getY()) {
                    if (fieldRelTurretPose.getX() >= GoalConstants.Red.kHub.getX()) {
                        goalLocation = GoalConstants.Blue.kFeedDepoRamp;

                    } else {
                        goalLocation = GoalConstants.Blue.kFeedDepo;

                    }
                } else {
                    if (fieldRelTurretPose.getX() >= GoalConstants.Red.kHub.getX()) {
                        goalLocation = GoalConstants.Blue.kFeedOutpostRamp;

                    } else {
                        goalLocation = GoalConstants.Blue.kFeedOutpost;

                    }
                }

                // NOTE: when turret Y == hub Y the denominator is 0 -> Infinity, so this
                // comparison is always true and m_dontFeed becomes true. Left as-is; flagging it.
                if (Math.abs(fieldRelTurretPose.getX() - GoalConstants.Blue.kHub.getX()) < ((36.0 / 39.37)
                        / (Math.abs(fieldRelTurretPose.getY() - GoalConstants.Blue.kHub.getY()) / 2.0))) {
                    m_dontFeed = true;
                }
            } else {
                if (fieldRelTurretPose.getY() < goalLocation.getY()) {
                    if (fieldRelTurretPose.getX() <= GoalConstants.Blue.kHub.getX()) {
                        goalLocation = GoalConstants.Red.kFeedDepoRamp;

                    } else {
                        goalLocation = GoalConstants.Red.kFeedDepo;

                    }
                } else {
                    if (fieldRelTurretPose.getX() <= GoalConstants.Blue.kHub.getX()) {
                        goalLocation = GoalConstants.Red.kFeedOutpostRamp;

                    } else {
                        goalLocation = GoalConstants.Red.kFeedOutpost;

                    }
                }

                if (Math.abs(fieldRelTurretPose.getX() - GoalConstants.Red.kHub.getX()) < ((36.0 / 39.37)
                        / (Math.abs(fieldRelTurretPose.getY() - GoalConstants.Red.kHub.getY()) / 2.0))) {
                    m_dontFeed = true;
                }
            }
        }

        // Field-relative speeds use the PREDICTED rotation for consistency with robotPose.
        ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelSpeeds, robotPose.getRotation());

        if (m_moveAndShot)
            goalLocation = compensateForMotion(goalLocation, fieldRelTurretPose, robotPose, fieldRelSpeeds);

        SmartDashboard.putNumber("Turret Location X", fieldRelTurretPose.getX());
        SmartDashboard.putNumber("Turret Location Y", fieldRelTurretPose.getY());

        Translation2d turretToGoal = goalLocation.minus(fieldRelTurretPose);
        Rotation2d alpha = turretToGoal.getAngle();
        double distance = turretToGoal.getNorm();

        SmartDashboard.putNumber("Distance to Goal", turretToGoal.getNorm());

        double turretSetpoint = alpha.getRadians() - robotPose.getRotation().getRadians();

        m_turret.setPosition(turretSetpoint);
        m_shooter.setShooterForDistance(distance, m_feedShot);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopRoller();
        m_shooter.setPosition(1.0);
        super.end(interrupted);
    }

    Translation2d compensateForMotion(Translation2d goalLocation, Translation2d fieldRelTurretPose, Pose2d robotPose,
            ChassisSpeeds fieldRelSpeeds) {

        Translation2d turretSpeedVector = new Translation2d(
                m_turret.getTranslation().getNorm() * fieldRelSpeeds.omegaRadiansPerSecond,
                new Rotation2d(robotPose.getRotation().getRadians() + Math.PI / 2.0
                        + m_turret.getTranslation().getAngle().getRadians()));

        ChassisSpeeds turretFieldSpeeds = new ChassisSpeeds(turretSpeedVector.getX(), turretSpeedVector.getY(), 0)
                .plus(fieldRelSpeeds);

        Translation2d finalGoalLocation = goalLocation;

        for (int i = 0; i < 10; i++) {
            // fieldRelTurretPose is already an absolute field-relative point -- do NOT add
            // robotPose.getTranslation() again (that double-counts the robot translation).
            Translation2d turretToGoal = finalGoalLocation.minus(fieldRelTurretPose);
            double distance = turretToGoal.getNorm();

            // Null-safe table lookup: degrade gracefully to the uncompensated goal rather
            // than NPE-ing on an empty table.
            Double shotTimeBoxed = m_feedShot
                    ? ShooterConstants.kFeedTimeTable.get(distance)
                    : ShooterConstants.kTimeTable.get(distance);
            if (shotTimeBoxed == null) {
                return goalLocation;
            }
            double shotTime = shotTimeBoxed;

            double offsetX = -1.0 * turretFieldSpeeds.vxMetersPerSecond * shotTime;
            double offsetY = -1.0 * turretFieldSpeeds.vyMetersPerSecond * shotTime;

            finalGoalLocation = goalLocation.plus(new Translation2d(offsetX, offsetY));
        }

        return finalGoalLocation;
    }

    public boolean isFeeding() {
        return m_feedShot;
    }

    public boolean dontFeed() {
        return m_dontFeed;
    }
}
