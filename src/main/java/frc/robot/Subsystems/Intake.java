package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

    private static double m_setPosition = 0.0;
    private static double m_setVoltage = 0.0;

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

    public void setPostion(double pose) {
        m_setPosition = pose;
        MathUtil.clamp(m_setPosition, IntakeConstants.kReverseLimit, IntakeConstants.kForwardLimit);
        m_arm.setControl(m_pvRequest.withPosition(m_setPosition));
    }

    public void setVoltage(double volts) {
        m_setVoltage = volts;
        if (atSetpoint())
            m_leftRoller.setVoltage(volts);
    }

    public void stopRoller() {
        m_leftRoller.stopMotor();
        m_rightRoller.stopMotor();
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public Command setIntake(double pose, double volts) {
        return runEnd(() -> {
            setPostion(pose);
            setVoltage(volts);
        }, () -> stopRoller());
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
        TalonFXConfigurator config = m_leftRoller.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kIntakeRollerStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeRollerSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

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
    }
}
