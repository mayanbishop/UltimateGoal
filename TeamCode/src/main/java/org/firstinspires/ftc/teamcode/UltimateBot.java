package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Created by Athira on 10/14/2018.
 */

public class UltimateBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private Navigation navigation = null;
    private VuforiaTracking vuforia = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildNavigation();
        startVuforiaSensing();

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
        this.navigation = new Navigation(robotHardware);
    }
    private void startVuforiaSensing()
    {
        this.vuforia = new VuforiaTracking(robotHardware);
        this.vuforia.initializeTracking();
    }
    public Navigation getNavigation(){return navigation;};
    public VuforiaTracking getVuforia(){return vuforia;}
    public RobotHardware getRobotHardware() {
        return robotHardware;
    }

}
