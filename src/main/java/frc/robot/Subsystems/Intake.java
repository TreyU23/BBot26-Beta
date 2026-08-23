package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX m_leftRoller = new TalonFX(IntakeConstants.kLeftRollerID);
    private final TalonFX m_rightRoller = new TalonFX(IntakeConstants.kRightRollerID);
    private final TalonFX m_arm = new TalonFX(IntakeConstants.kArmID);

    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);

    private double m_setPosition = 0.0;
    private double m_setVoltage = 0.0;

    public Intake() {
        rollerConfigs();
        armConfigs();
    }

    public double getPosition() {
        return m_arm.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_leftRoller.getVelocity().getValueAsDouble();
    }

    public double getArmStatorCurrent() {
        return m_arm.getStatorCurrent().getValueAsDouble();
    }

    public double getArmSupplyCurrent() {
        return m_arm.getSupplyCurrent().getValueAsDouble();
    }

    public double getRollerStatorCurrent() {
        return m_leftRoller.getStatorCurrent().getValueAsDouble();
    }

    public double getRollerSupplyCurrent() {
        return m_leftRoller.getSupplyCurrent().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(m_setPosition, getPosition(), IntakeConstants.kPositionTolerance);
    }

    public void setPosition(double pose) {
        m_setPosition = MathUtil.clamp(pose, IntakeConstants.kReverseLimit, IntakeConstants.kForwardLimit);
        m_arm.setControl(m_pvRequest.withPosition(m_setPosition));
    }

    public void setVoltage(double volts) {
        if (atSetpoint()) {
            m_setVoltage = volts;
            m_leftRoller.setVoltage(volts);
        } else {
            m_setVoltage = 0.0;
            m_leftRoller.stopMotor();
        }
    }

    public void stopRoller() {
        // m_rightRoller mirrors m_leftRoller via a persistent Follower request set once in
        // rollerConfigs(); commanding it directly here would overwrite that request and
        // permanently break the follow relationship, so only the master needs to be stopped.
        m_leftRoller.stopMotor();
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public Command setIntake(double pose, double volts) {
        return runEnd(() -> {
            setPosition(pose);
            setVoltage(volts);
        }, () -> stopRoller());
    }

    public Command zeroIntakeCmd() {
        return runEnd(() -> m_arm.setVoltage(1.0), () -> {
            stopArm();
            m_arm.setPosition(0.0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        SmartDashboard.putNumber("Intake Set Position", m_setPosition);
        SmartDashboard.putNumber("Intake Set Voltage", m_setVoltage);
        SmartDashboard.putBoolean("Intake At Setpoint", atSetpoint());

        SmartDashboard.putNumber("Intake Arm Stator Current", getArmStatorCurrent());
        SmartDashboard.putNumber("Intake Arm Supply Current", getArmSupplyCurrent());
        SmartDashboard.putNumber("Intake Left Stator Current", getRollerStatorCurrent());
        SmartDashboard.putNumber("Intake Left Supply Current", getRollerSupplyCurrent());

        SignalLogger.writeDouble("Intake Set Position", m_setPosition);
        SignalLogger.writeDouble("Intake Set Voltage", m_setVoltage);
        SignalLogger.writeBoolean("Intake At Setpoint", atSetpoint());
    }

    public void rollerConfigs() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kIntakeRollerStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeRollerSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs neutralMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

        TalonFXConfigurator leftConfig = m_leftRoller.getConfigurator();
        leftConfig.apply(currentLimits);
        leftConfig.apply(neutralMode);

        TalonFXConfigurator rightConfig = m_rightRoller.getConfigurator();
        rightConfig.apply(currentLimits);
        rightConfig.apply(neutralMode);

        m_rightRoller.setControl(new Follower(IntakeConstants.kLeftRollerID, IntakeConstants.kRollerAligned));
    }

    public void armConfigs() {
        TalonFXConfigurator config = m_arm.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kIntakeArmStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeArmSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        config.apply(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(IntakeConstants.kForwardLimit)
                .withReverseSoftLimitThreshold(IntakeConstants.kReverseLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true));

        config.apply(new Slot0Configs()
                .withKP(IntakeConstants.kP)
                .withKS(IntakeConstants.kS)
                .withKG(IntakeConstants.kG)
                .withKV(IntakeConstants.kV)
                .withGravityType(GravityTypeValue.Arm_Cosine));

        config.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio));
    }
}
