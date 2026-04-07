package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerToShooterSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);
    private boolean m_running = false;

    public RollerToShooterSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.ROLLER_TO_SHOOTER, frc.robot.generated.TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("RollerToShooter/Running", m_running);
    }

    public void start() {
        m_running = true;
        m_motor.setControl(m_voltageOut.withOutput(Constants.RollerToShooterConstants.VOLTAGE));
    }

    public void toggle() {
        m_running = !m_running;
        if (m_running) {
            m_motor.setControl(m_voltageOut.withOutput(Constants.RollerToShooterConstants.VOLTAGE));
        } else {
            m_motor.setControl(m_voltageOut.withOutput(0.0));
        }
    }

    public void stop() {
        m_running = false;
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public boolean isRunning() {
        return m_running;
    }
}
