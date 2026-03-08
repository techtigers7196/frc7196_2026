package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import java.security.Key;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
/**
 * Shooter subsystem: encoder, PID feedback, feedforward, and NetworkTables telemetry.
 * PID and feedforward are initialized to 0.0 (tunable later in Constants or NetworkTables).
 */
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotor = new SparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();//new Encoder(0, 1, false);

  // Feedforward and PID (start at 0.0 per request)
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.00, 0.000187);//(1/0.18)/60);
  private final PIDController m_shooterPID = new PIDController(0.0004, 0.0000, 0.00001);


  // NetworkTables telemetry (units: RPM)
  private final NetworkTable m_nt = NetworkTableInstance.getDefault().getTable("Shooter");
  private final NetworkTableEntry m_rpmEntry = m_nt.getEntry("rpmRPM");
  private final NetworkTableEntry m_setpointEntry = m_nt.getEntry("setpointRPM");
  private final NetworkTableEntry m_errorEntry = m_nt.getEntry("RPMerror");
  private final NetworkTableEntry m_voltageEntry = m_nt.getEntry("applied voltage");
  private final NetworkTableEntry m_ready = m_nt.getEntry("shooter ready");

  private final BangBangController m_controller = new BangBangController();

  private double m_setpoint = 2000.0; // RPM
  // private double bb_limit = m_setpoint * 0.9;
  private final double m_limit = 500;



  public ShooterSubsystem() {
    // sensible defaults so code compiles even without external Constants
    //m_shooterEncoder.setDistancePerPulse(1.0);
    m_shooterPID.setTolerance(0.0);

    // initialize NetworkTables
    m_rpmEntry.setDouble(0.0);
    m_setpointEntry.setDouble(0.0);
  
    // m_pEntry.setDouble(0);
    // m_iEntry.setDouble(0);
    // m_dEntry.setDouble(0);  
    // m_ksEntry.setDouble(m_ks);
    Shuffleboard.getTab("Shooter").add("P", m_shooterPID.getP());
    Shuffleboard.getTab("Shooter").add("I", m_shooterPID.getI());
    Shuffleboard.getTab("Shooter").add("D", m_shooterPID.getD());
    Shuffleboard.getTab("Shooter").add("ks", m_shooterFeedforward.getKs());
    Shuffleboard.getTab("Shooter").add("kv", m_shooterFeedforward.getKv());

    // default command: disable motor (idle)
    //setDefaultCommand(Commands.runOnce(() -> m_shooterMotor.disable()).andThen(Commands.run(() -> {})).withName("ShooterIdle"));
  }
  
  /**
   * Command that runs the shooter to the provided setpoint (RPS). The command continuously
   * applies feedforward + PID feedback to the motor. The parameter is a DoubleSupplier so
   * callers can provide a constant or a dynamic source.
   */
  public Command runShootCommand() {
    return Commands.run(() -> {
      double setpoint = m_setpoint; // expected in RPM
      double currentRPM = m_shooterEncoder.getVelocity();
      // double error = Math.abs(setpoint - currentRPM);
      double limit = m_setpoint * 1;
      // Bang Bang + Feedforward
      if (false){
        // double outputVoltage = m_controller.calculate(currentRPM, setpoint) * 11 +  0.9 *m_shooterFeedforward.calculate(setpoint);
        double outputVoltage = m_controller.calculate(currentRPM, setpoint) * 11;
        m_shooterMotor.setVoltage(outputVoltage);
        SmartDashboard.putNumber("bbvoltage",outputVoltage);      
      }

      else{
        // PID + feedforward
        double pidOutput = m_shooterPID.calculate(currentRPM, m_setpoint);

        double ff = m_shooterFeedforward.calculate(m_setpoint);
        double  outputVoltage = ff + pidOutput ;
        m_shooterMotor.set(outputVoltage);
        SmartDashboard.putNumber("ShooterRPM",currentRPM);
      }
      SmartDashboard.putBoolean("Is PID active", currentRPM >= limit);
    }, this).withName("RunShooter");
  }

  public Command setPower(double power)
  {
    return run(() -> {
      m_shooterMotor.set(power);
    });
  }

  public Command stop()
  {
    return run(() -> {
      m_shooterMotor.set(0);
    });
  }

  public void increment_setpoint(double increment)
  {
    m_setpoint += increment;
  }

  public void set_setpoint(double setpoint)
  {
    m_setpoint = setpoint;
  }

  @Override
  public void periodic() {
    // publish encoder rate (RPM) and current setpoint
    double rpm = m_shooterEncoder.getVelocity(); // convert rev/sec to RPM
    m_rpmEntry.setDouble(rpm);
    m_setpointEntry.setDouble(m_setpoint);
    m_errorEntry.setDouble(m_setpoint - rpm);
    m_voltageEntry.setDouble(m_shooterMotor.getAppliedOutput() * m_shooterMotor.getBusVoltage());

    if (rpm < m_setpoint - 10)
    {
      m_ready.setBoolean(false);
    } 
    else
    {
     m_ready.setBoolean(true);
    }
    // double p = m_pEntry.getDouble(m_shooterPID.getP());
    // double i = m_pEntry.getDouble(m_shooterPID.getI());
    // double d = m_dEntry.getDouble(m_shooterPID.getD());
    // m_shooterPID.setP(p);
    // m_shooterPID.setI(i);
    // m_shooterPID.setD(d);
    // Also publish to SmartDashboard for easy graphing
    SmartDashboard.putNumber("Shooter/rpmRPM", rpm);
    SmartDashboard.putNumber("Shooter/setpointRPM", m_setpoint);
    SmartDashboard.putNumber("Shooter/RPMerror", m_setpoint - rpm);
    // SmartDashboard.putNumber("Shooter/P", p);
    // SmartDashboard.putNumber("Shooter/I", i);
    // SmartDashboard.putNumber("Shooter/D", d);
    // SmartDashboard.putNumber("Shooter/ks", m_ks);




  }
}

