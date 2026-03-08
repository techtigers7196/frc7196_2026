package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakemotor = new SparkMax(16, MotorType.kBrushless);

    public Command runIntake(double power)
    {
        return run (() -> {
            m_intakemotor.set(power);
        });
    }

    public Command stop () {
        return run (() -> {
            m_intakemotor.set(0);
        });

    }
      
}

