package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax m_hoppermotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_feedermotor = new SparkMax(10, MotorType.kBrushless);

    private final NetworkTable m_nt = NetworkTableInstance.getDefault().getTable("Hopper");
    private final NetworkTableEntry m_timerEntry = m_nt.getEntry("timer");

    public Command runHopper(double hopperpower, double feedpower)
    {
        return runOnce(() -> {
            m_hoppermotor.set(hopperpower);
            m_feedermotor.set(feedpower);
        });
    }

    Timer timer = new Timer();
    
    public Command runHopperifReady(double hopperpower, double feedpower, ShooterSubsystem shooter)
    {
        return run(() -> {
            if (shooter.isReady() && !timer.isRunning())
            {
                timer.start();
            }
            else if (shooter.isReady() && timer.get() >= 0.2)
            {
                m_hoppermotor.set(hopperpower);
                m_feedermotor.set(feedpower);
            }
            else if (!shooter.isReady())
            {
                timer.stop();
                timer.reset();
                m_hoppermotor.set(0);
                m_feedermotor.set(0);
            }
        });
    }
    public Command stop() {
        return runOnce(() ->{
            timer.stop();
            timer.reset();
            m_hoppermotor.set(0);
            m_feedermotor.set(0);
        });
    }


    public void periodic()
    {
        m_timerEntry.setDouble(timer.get());
    }

}
