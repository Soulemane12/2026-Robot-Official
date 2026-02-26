package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private double m_targetRotations = 0.0;

    public IntakeSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.INTAKE_PIVOT, frc.robot.generated.TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Slot0 PID gains for MotionMagic — tune these on the real robot
        Slot0Configs slot0 = new Slot0Configs()
            .withKP(2.0)    // Proportional gain (lowered to reduce harshness)
            .withKI(0.0)    // Integral gain
            .withKD(0.02)   // Derivative gain
            .withKS(0.25)   // Static feedforward (volts to overcome friction)
            .withKV(0.18)   // Velocity feedforward (increased so feedforward does more work)
            .withKA(0.01);  // Acceleration feedforward
        config.Slot0 = slot0;

        MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(140.0)
            .withMotionMagicAcceleration(280.0)
            .withMotionMagicJerk(1400.0);
        config.MotionMagic = mmConfig;

        m_motor.getConfigurator().apply(config);

        // Zero the encoder on startup so current position becomes 0
        m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PosRot", getPositionRotations());
        SmartDashboard.putNumber("Intake/TargetRot", m_targetRotations);
        SmartDashboard.putNumber("Intake/VelocityRPS", getVelocityRPS());
    }

    public void zero() {
        m_motor.setPosition(0.0);
        m_targetRotations = 0.0;
    }

    public void goTo(double rotations) {
        m_targetRotations = rotations;
        m_motor.setControl(m_motionMagic.withPosition(rotations));
    }

    public void jogVolts(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public void stop() {
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getVelocityRPS() {
        return m_motor.getVelocity().getValueAsDouble();
    }
}
