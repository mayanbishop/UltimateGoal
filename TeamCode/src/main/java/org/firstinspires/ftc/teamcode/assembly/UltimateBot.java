package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Athira on 10/14/2018.
 */

public class UltimateBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private SensorNavigation navigation = null;
    private VisualCortex vuforia_tf = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildNavigation();
        initializeVuforiaAndTensorFlow();

    }
    public void buildChassisAssembly () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }
    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }
    public void buildNavigation()
    {
        this.navigation = new SensorNavigation(robotHardware);
    }
    private void initializeVuforiaAndTensorFlow()
    {
        this.vuforia_tf = new VisualCortex(robotHardware);
        vuforia_tf.initVuforia();
        vuforia_tf.initTfod();
        vuforia_tf.loadTrackables();
    }
    public SensorNavigation getNavigation(){return navigation;};
    public VisualCortex getVisualCortex(){return vuforia_tf;}
    public RobotHardware getRobotHardware() {
        return robotHardware;
    }

}
