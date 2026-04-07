package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.CANIds.INDEXER_MOTOR, TunerConstants.kCANBus);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);
    private boolean m_running = false;

    public void start() {
        m_running = true;
        m_motor.setControl(m_voltageOut.withOutput(Constants.IndexerConstants.VOLTAGE));
    }

    public void stop() {
        m_running = false;
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public void toggle() {
        if (m_running) stop(); else start();
    }

    public boolean isRunning() {
        return m_running;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer/Running", m_running);
    }
}
