package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax m_hoppermotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_feedermotor = new SparkMax(10, MotorType.kBrushless);

    public Command runHopper(double hopperpower, double feedpower)
    {
        return run(() -> {
            m_hoppermotor.set(hopperpower);
            m_feedermotor.set(feedpower);
        });
    }

    public Command stop() {
        return run(() ->{
            m_hoppermotor.set(0);
            m_feedermotor.set(0);
        });
    }


}
