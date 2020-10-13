package org.firstinspires.ftc.teamcode.assembly;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class RobotHardware
{
    //Chassis
    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    //Webcam
    public WebcamName webcam = null;

    public BNO055IMU imu;

    //Distance Sensors
    Rev2mDistanceSensor frontRightSensor = null;
    Rev2mDistanceSensor backRightSensor = null;
    Rev2mDistanceSensor frontLeftSensor = null;
    Rev2mDistanceSensor backLeftSensor = null;

    Rev2mDistanceSensor backLaser = null;
    Rev2mDistanceSensor frontLaser = null;

    //Adding the Hardware Map
    private HardwareMap hwMap  = null;

    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        //Wheel motors
        frontLeftWheel = hwMap.get(DcMotor.class, "frontLeft");
        frontRightWheel = hwMap.get(DcMotor.class, "frontRight");
        backLeftWheel = hwMap.get(DcMotor.class, "backLeft");
        backRightWheel = hwMap.get(DcMotor.class, "backRight");

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        frontLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        webcam = hwMap.get(WebcamName.class, "webcam");
        imu = hwMap.get(BNO055IMU.class, "imu");
/*
        frontRightSensor = hwMap.get(Rev2mDistanceSensor.class, "frontRightSensor");
        backRightSensor = hwMap.get(Rev2mDistanceSensor.class, "backRightSensor");

        frontLeftSensor = hwMap.get(Rev2mDistanceSensor.class, "frontLeftSensor");
        backLeftSensor = hwMap.get(Rev2mDistanceSensor.class, "backLeftSensor");

        backLaser = hwMap.get(Rev2mDistanceSensor.class, "backSensor");
        frontLaser = hwMap.get(Rev2mDistanceSensor.class, "frontSensor");
*/

    }


    public HardwareMap getHwMap() {
        return hwMap;
    }
}