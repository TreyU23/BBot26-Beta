package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.KickerConstants;

public class Kicker extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(KickerConstants.kMotorID);

    private final VoltageOut m_voRequest = new VoltageOut(0.0);
    private final VelocityVoltage m_vvRequest = new VelocityVoltage(0.0);

    private double m_setpoint = 0.0;
    
    public Kicker() {
        motorConfigs();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    public void setVoltage(double volts) {
        m_setpoint = volts;
        m_motor.setControl(m_voRequest.withOutput(m_setpoint));
    }

    public void setVelocity(double velocity) {
        m_setpoint = velocity;
        m_motor.setControl(m_vvRequest.withVelocity(m_setpoint));
    }

    public void stop() {
        m_setpoint = 0.0;
        m_motor.stopMotor();
    }

    public Command setVoltageCmd(double volts) {
        return runEnd(()-> setVoltage(volts), ()-> stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Kicker Supply Current", getSupplyCurrent());
        SmartDashboard.putNumber("Kicker Velocity", getVelocity());
        SmartDashboard.putNumber("Kicker Setpoint", m_setpoint);

        SignalLogger.writeDouble("Kicker Setpoint", m_setpoint);
    }

    public void motorConfigs() {
        TalonFXConfigurator config = m_motor.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kKickerStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kKickerSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new Slot0Configs()
                .withKP(KickerConstants.kP)
                .withKS(KickerConstants.kS)
                .withKV(KickerConstants.kV));

        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }
}
