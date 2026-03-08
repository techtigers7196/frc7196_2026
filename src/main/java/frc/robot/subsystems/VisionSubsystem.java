package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
    //private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight_left").getEntry("camerapose_targtespace").getDoubleArray(null);
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tz = table.getEntry("tz");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry pitch = table.getEntry("pitch");
    private NetworkTableEntry yaw = table.getEntry("yaw");
    private NetworkTableEntry roll = table.getEntry("roll");

    private final NetworkTable m_nt = NetworkTableInstance.getDefault().getTable("ll-distance");
    private final NetworkTableEntry m_distancEntry = m_nt.getEntry("Distance");


    // private NetworkTableEntry ta = table.getEntry("ta");

    public double[] getXYA()
    {
        double x = tx.getDouble(0.0);
        double y  = ty.getDouble(0.0);
        double z = tz.getDouble(0.0);
        double v = tv.getDouble(0);
        /*double pitch = pitch.getDouble(0.0);
        double yaw  = yaw.getDouble(0.0);
        double roll = roll.getDouble(0.0);*/
     

        
        return new double[]{x, y, z, v};
    }
    
    public double getDistance()
    {
        double[] value = getXYA();

        double a2 = value[1];
        double a1 = 20;
        double h2 = 1.12395;
        double h1 = 0.5184902;
        double d = (h2-h1)/Math.tan((a1+a2) * 3.14159 / 180);

        if (value[3] == 0)
        {
            return 0;
        }

        return d;

    }

    public double getrpm(){
        double distance = getDistance();
        if (distance !=0){
            return 10.37056*distance*distance+81.9512*distance+1618.04+35;
        }
    return 1850;
    }

    public void periodic()
    {
        double distance = getDistance();
        if (distance != 0)
        {
            m_distancEntry.setDouble(getDistance());
        }
    }
}
