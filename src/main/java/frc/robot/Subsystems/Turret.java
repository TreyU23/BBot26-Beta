package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.TurretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(TurretConstants.kMotorID);

    private final CANcoder m_leftCANCoder = new CANcoder(TurretConstants.kLeftCANCoderID);
    private final CANcoder m_rightCANCoder = new CANcoder(TurretConstants.kRightCANCoderID);

    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);

    private static double m_setpoint = 0.0;
    private static double m_lastSetpoint = 0.0;
    
    private DoubleSupplier m_feedForward;

    private EasyCRT m_easyCRT;
    private Angle m_CRTOutput = Rotations.of(0.0);

    public Turret(DoubleSupplier robotRotation) {
        m_feedForward = ()-> (robotRotation.getAsDouble() * 1.1 / (2.0 * Math.PI) * TurretConstants.kV);

        notSoEasyCRT();
        CANCoderConfigs();
        motorConfigs();
        setStartingPose();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(m_setpoint, getPosition(), TurretConstants.kTolerance);
    }

    public void setPosition(double Angle) {
        m_setpoint = wrapAngle(Angle / (2.0 * Math.PI));
        m_setpoint = chooseSetpoint(m_setpoint, m_lastSetpoint);
    }

    public void stop() {
        m_setpoint = getPosition();
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        m_motor.setControl(m_pvRequest
            .withFeedForward(Voltage.ofBaseUnits(m_feedForward.getAsDouble(), Volts)));

        SmartDashboard.putNumber("Turret Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Turret Supply Current", getSupplyCurrent());
        SmartDashboard.putNumber("Turret Position", getPosition());
        SmartDashboard.putNumber("Turret Setpoint", m_setpoint);

        SmartDashboard.putNumber("Turret Left CANCoder Position", m_leftCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Right CANCoder Position", m_rightCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Left CANCoder Rel Position", m_leftCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Right CANCoder Rel Position", m_rightCANCoder.getPosition().getValueAsDouble());

        SignalLogger.writeDouble("Turret Setpoint", m_setpoint);

        m_lastSetpoint = m_setpoint;
    }

    public void notSoEasyCRT() {
        Supplier<Angle> leftCANCoderPose = () -> m_leftCANCoder.getAbsolutePosition().getValue();
        Supplier<Angle> RightCANCoderPose = () -> m_rightCANCoder.getAbsolutePosition().getValue();

        EasyCRTConfig Config = new EasyCRTConfig(leftCANCoderPose, RightCANCoderPose)
                .withEncoderRatios(TurretConstants.kCRTGearRatio1, TurretConstants.kCRTGearRatio2)
                .withMechanismRange(Rotations.of(TurretConstants.kReverseLimit),
                        Rotations.of(TurretConstants.kForwardLimit))
                .withAbsoluteEncoderInversions(false, false)
                .withMatchTolerance(Rotations.of(TurretConstants.kCRTTolerance));

        m_easyCRT = new EasyCRT(Config);
    }

    public void setStartingPose() {
        SmartDashboard.putBoolean("CRT Set", false);

        m_easyCRT.getAngleOptional().ifPresent(mechAngle -> {
            SmartDashboard.putNumber("CRT Solve", mechAngle.magnitude());
            m_CRTOutput = mechAngle;
            SmartDashboard.putBoolean("CRT Set", true);
        });

        m_motor.getConfigurator().setPosition(m_CRTOutput);
        m_setpoint = getPosition();
    }

    public double wrapAngle(double angleRotations) {
        double w = angleRotations - Math.floor(angleRotations);
        if (w >= 0.5)
            w -= 1.0;
        return w;
    }

    public double chooseSetpoint(double wrappedSetpoint, double lastSetpoint) {
        double best = Double.NaN;
        double bestAbsErr = Double.POSITIVE_INFINITY;

        for (int k = -1; k <= 1; k++) {
            double candidate = wrappedSetpoint + k;

            if (candidate < TurretConstants.kReverseLimit || candidate > TurretConstants.kForwardLimit)
                continue;

            double err = candidate - lastSetpoint;
            double absErr = Math.abs(err);

            if (absErr < bestAbsErr) {
                bestAbsErr = absErr;
                best = candidate;
            }
        }

        if (Double.isNaN(best)) {
            best = MathUtil.clamp(best, TurretConstants.kReverseLimit, TurretConstants.kForwardLimit);
        }

        return best;
    }

    public void CANCoderConfigs() {
        m_leftCANCoder.getConfigurator().apply(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConstants.kLeftMagnetOffset)
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        m_rightCANCoder.getConfigurator().apply(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConstants.kRightMagnetOffset)
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }

    public void motorConfigs() {
        TalonFXConfigurator config = m_motor.getConfigurator();

         config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kTurretStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kTurretSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new Slot0Configs()
                .withKP(TurretConstants.kP));

        config.apply(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(TurretConstants.kForwardLimit)
                .withReverseSoftLimitThreshold(TurretConstants.kReverseLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true));

        config.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(TurretConstants.kGearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        config.apply(new VoltageConfigs()
                .withPeakForwardVoltage(TurretConstants.kPeakForwardVolts)
                .withPeakReverseVoltage(TurretConstants.kPeakReverseVolts));

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }
}
