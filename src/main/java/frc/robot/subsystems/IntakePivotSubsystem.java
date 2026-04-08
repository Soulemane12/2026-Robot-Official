package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivotSubsystem extends SubsystemBase {
    private static final double DEPLOYED_TARGET_MIN_ROT = 5.0;
    private static final double DEPLOYED_STALL_MARGIN_ROT = 0.08;
    private static final double LOWER_LIMIT_ROT = 0.67;
    private static final double STOW_STOP_MARGIN_ROT = 0.20;
    private static final double STALL_VELOCITY_RPS = 1.0;
    private static final double STALL_CURRENT_A = 30.0;

    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private double m_targetRotations = 0.0;
    private boolean m_limitsEnabled = true;
    private boolean m_positionControlActive = false;

    public IntakePivotSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.INTAKE_PIVOT, frc.robot.generated.TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Slot0 PID gains for MotionMagic — tune these on the real robot
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(3.0)    // Proportional gain
            .withKI(0.0)    // Integral gain
            .withKD(0.08)   // Derivative gain to reduce overshoot
            .withKS(0.25)   // Static feedforward (volts to overcome friction)
            .withKV(0.18)   // Velocity feedforward (increased so feedforward does more work)
            .withKA(0.01);  // Acceleration feedforward
        config.Slot0 = slot0;

        MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(30.0)
            .withMotionMagicAcceleration(60.0)
            .withMotionMagicJerk(300.0);
        config.MotionMagic = mmConfig;

        // Current limits to prevent stalling against hard stops
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        m_motor.getConfigurator().apply(config);

        // Zero the encoder on startup so current position becomes 0
        m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        if (m_limitsEnabled) {
            boolean stalled = getStatorCurrentA() >= STALL_CURRENT_A
                    && Math.abs(getVelocityRPS()) <= STALL_VELOCITY_RPS;

            // Stop at upper hard stop
            if (getPositionRotations() >= 101 && getVelocityRPS() > 0.0) {
                stop();
            }
            // Stop at lower hard stop (stow) to prevent stalling.
            // Triggers whether motor is still moving toward stop (vel < 0)
            // OR holding against it with MotionMagic (vel ≈ 0).
            // Guard: only when targeting stow so deploying still works.
            if (m_positionControlActive
                    && m_targetRotations <= 1.0
                    && (getPositionRotations() <= m_targetRotations + STOW_STOP_MARGIN_ROT || stalled)) {
                stop();
            }

            // Stop near the deployed target if the pivot has slowed or overshot, so MotionMagic
            // does not keep pushing the mechanism into a hard stop.
            if (m_positionControlActive
                    && m_targetRotations >= DEPLOYED_TARGET_MIN_ROT
                    && ((getPositionRotations() >= m_targetRotations - DEPLOYED_STALL_MARGIN_ROT
                            && Math.abs(getVelocityRPS()) <= STALL_VELOCITY_RPS)
                        || stalled)) {
                stop();
            }
        }

        SmartDashboard.putNumber("Intake/PosRot", getPositionRotations());
        SmartDashboard.putNumber("Intake/TargetRot", m_targetRotations);
        SmartDashboard.putNumber("Intake/VelocityRPS", getVelocityRPS());
        SmartDashboard.putNumber("Intake/StatorCurrentA", getStatorCurrentA());
        SmartDashboard.putNumber("Intake/SupplyCurrentA", getSupplyCurrentA());
        SmartDashboard.putBoolean("Intake/LimitsEnabled", m_limitsEnabled);
        SmartDashboard.putBoolean("Intake/PositionControlActive", m_positionControlActive);
        SmartDashboard.putBoolean("Intake/IsOut", getPositionRotations() > Constants.IntakeConstants.STOW + 5.0);
    }

    public void zero() {
        m_motor.setPosition(0.0);
        m_targetRotations = 0.0;
        m_positionControlActive = false;
    }

    public void toggleLimits() {
        m_limitsEnabled = !m_limitsEnabled;
    }

    public void goTo(double rotations) {
        if (m_limitsEnabled) {
            m_targetRotations = Math.max(LOWER_LIMIT_ROT, Math.min(101.0, rotations));
        } else {
            m_targetRotations = rotations;
        }
        m_positionControlActive = true;
        m_motor.setControl(m_motionMagic.withPosition(m_targetRotations));
    }

    public void jogVolts(double volts) {
        m_positionControlActive = false;
        if (m_limitsEnabled) {
            if (volts > 0.0 && getPositionRotations() >= 101) {
                stop();
                return;
            }
        }
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public void stop() {
        m_positionControlActive = false;
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getVelocityRPS() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    public double getStatorCurrentA() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrentA() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }
}
