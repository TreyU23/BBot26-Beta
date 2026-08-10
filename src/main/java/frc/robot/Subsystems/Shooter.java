package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_hood = new TalonFX(ShooterConstants.Hood.kMotorID);
    private final TalonFX m_roller = new TalonFX(ShooterConstants.Roller.kMotorID);

    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);
    private final VelocityVoltage m_vvRequest = new VelocityVoltage(0.0);

    private static double m_setPosition = 0.0;
    private static double m_setVelocity = 0.0;

    private static boolean m_allowHood = false;

    public Shooter() {
        hoodConfigs();
        rollerConfigs();
    }

    public double getPosition() {
        return m_hood.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_roller.getVelocity().getValueAsDouble();
    }

    public double getHoodStatorCurrent() {
        return m_hood.getStatorCurrent().getValueAsDouble();
    }

    public double getHoodSupplyCurrent() {
        return m_hood.getSupplyCurrent().getValueAsDouble();
    }

    public double getRollerStatorCurrent() {
        return m_roller.getStatorCurrent().getValueAsDouble();
    }

    public double getRollerSupplyCurrent() {
        return m_roller.getSupplyCurrent().getValueAsDouble();
    }

    public void allowHood(boolean allowHood) {
        m_allowHood = allowHood;
        if (!m_allowHood)
            setPosition(0.0);
    }

    public void setPosition(double pose) {
        if (m_allowHood) {
            m_setPosition 
                = MathUtil.clamp(pose, ShooterConstants.Hood.kReverseLimit, ShooterConstants.Hood.kForwardLimit);
        } else {
            m_setPosition = 0.0;
        }

        m_hood.setControl(m_pvRequest.withPosition(m_setPosition));
    }

    public void setVelocity(double velocity) {
        m_setVelocity = velocity;
        m_roller.setControl(m_vvRequest.withVelocity(m_setVelocity));
    }

    public void stopHood() {
        setPosition(getPosition());
    }

    public void stopRoller() {
        m_setVelocity = 0.0;
        m_roller.stopMotor();
    }

    public void setShooterForDistance(double distance, boolean isFeeding) {
        if (isFeeding) {
            setPosition(ShooterConstants.kPositionFeedTable.get(distance));
            setVelocity(ShooterConstants.kVelocityFeedTable.get(distance));
        } else {
            setPosition(ShooterConstants.kPositionTable.get(distance));
            setVelocity(ShooterConstants.kVelocityTable.get(distance));
        }
    }

    public Command setShooter(double pose, double velocity) {
        return runEnd(()-> {
            setPosition(pose);
            setVelocity(velocity);
        }, ()-> {
            setPosition(0.0);
            stopRoller();
        });
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Position", getPosition());
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        SmartDashboard.putNumber("Shooter Set Position", m_setPosition);
        SmartDashboard.putNumber("Shooter Set Velocity", m_setVelocity);

        SmartDashboard.putNumber("Shooter Hood Stator Current", getHoodStatorCurrent());
        SmartDashboard.putNumber("Shooter Hood Supply Current", getHoodSupplyCurrent());
        SmartDashboard.putNumber("Shooter Roller Stator Current", getRollerStatorCurrent());
        SmartDashboard.putNumber("Shooter Roller Supply Current", getRollerSupplyCurrent());

        SignalLogger.writeDouble("Shooter Set Position", m_setPosition);
        SignalLogger.writeDouble("Shooter Set Velocity", m_setVelocity);
    }

    public void rollerConfigs() {
        TalonFXConfigurator config = m_roller.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kShooterRollerStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kShooterRollerSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        config.apply(new Slot0Configs()
                .withKP(ShooterConstants.Roller.kP)
                .withKS(ShooterConstants.Roller.kS)
                .withKV(ShooterConstants.Roller.kV));
    }

    public void hoodConfigs() {
        TalonFXConfigurator config = m_hood.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kShooterHoodStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kShooterHoodSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        config.apply(new Slot0Configs()
                .withKP(ShooterConstants.Hood.kP)
                .withKS(ShooterConstants.Hood.kS)
                .withKV(ShooterConstants.Hood.kV));

        config.apply(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(ShooterConstants.Hood.kForwardLimit)
                .withReverseSoftLimitThreshold(ShooterConstants.Hood.kReverseLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true));
    }
}
