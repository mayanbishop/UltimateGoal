package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.assembly.ChassisAssembly;
import org.firstinspires.ftc.teamcode.assembly.SensorNavigation;
import org.firstinspires.ftc.teamcode.assembly.VisualCortex;
import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

import java.util.ArrayList;
import java.util.List;




@Autonomous(name = "BlueRight", group = "Qualifier")
public class BlueRight extends LinearOpMode
{

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 50;
    final double COUNTS_PER_DEGREE = 8.5;
    double PUSHER_POS = 0;

    //target constants
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Servo Positions


    //Movement Constants
    final double WHEEL_SPEED = 1;
    final double SIDE_SHIFT = 6;

    //Creating a  robot object
    UltimateBot ultimateBot = new UltimateBot();
    VisualCortex vcortex = null;
    SensorNavigation nav = null;
    ChassisAssembly chassis = null;;
    List<VuforiaTrackable> allTrackables = null;


    //Time
    ElapsedTime runtime = new ElapsedTime();
    int numRings = 0;

    @Override public void runOpMode()
    {

        //Intialize Robot
        ultimateBot.initRobot(hardwareMap);
        nav = ultimateBot.getNavigation();
        chassis = ultimateBot.getChassisAssembly();;

        //Initilize Vuforia and tensor flow
        ultimateBot.initializeVuforiaAndTensorFlow();
        ultimateBot.loadVuforiaTrackables();

        vcortex = ultimateBot.getVisualCortex();
        //allTrackables = vcortex.getAllTrackables();

        //Wait for Start
        telemetry.addData("Waiting for start", "");
        ArrayList<Integer> stackArrayList=new ArrayList<Integer>();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Scanning stack ", "..Waiting for Start");
            telemetry.update();
            int stack = vcortex.checkStarterStack(3);
            stackArrayList.add(new Integer(stack));
            // if there are more than 10 scans, throw away the first five scans from the list
            if(stackArrayList.size() > 10)
            {
                stackArrayList.subList(0, 5).clear();
            }
        }
        //Not needed as we scan during Init
        //waitForStart();

        /**
         * DURING RUNTIME
         */
        telemetry.addData("Starting Autonomous ", "");
        String zone =   getZone(stackArrayList, 5);
        sleep(100000);

        /*
        rampSpeedEncoderDrive(1.0, 40, 3);
        encoderTurn(0.5, -79, 3);
        encoderDrive(1.0, 15, 3);
        ultimateBot.getIntakeAssembly().intake();
        sleep(1000);
        encoderDrive(1.0, 5, 3);
        sleep(1000);
        encoderDrive(1.0, 5, 3);
        sleep(1000);
        encoderDrive(1.0, 5, 3);
        ultimateBot.getIntakeAssembly().stopIntake();
        encoderTurn(0.5, 79, 3);
        rampSpeedEncoderDrive(1.0, 60, 5);
     */
    }

    public String  getZone(ArrayList<Integer> stack, int numOfScans)
    {
        int totalTries = stack.size();
        telemetry.addData("totalTries ", totalTries);
        String zone = "A";
        int singleCount= 0;
        int noneCount = 0;
        int quadCount = 0;

        int scansCount = 0;
        if(totalTries < numOfScans)
        {
            scansCount = totalTries;
        }
        else
        {
            scansCount= numOfScans;
        }

        for(int i= totalTries- 1; i >= totalTries - scansCount ; i--)
        {
            if(stack.get(i) ==0) { noneCount++;  }
            else if(stack.get(i) ==1)  { singleCount++;   }
            else if(stack.get(i) ==4)  { quadCount++;    }

        }
        telemetry.addData("noneCount ", noneCount);
        telemetry.addData("singleCount ", singleCount);
        telemetry.addData("quadCount ", quadCount);
        telemetry.update();
        if( noneCount >= singleCount && noneCount >= quadCount) {
            telemetry.addData("No Ring Score  is " + noneCount + " out of last " + scansCount , "  Zone A") ;
            zone ="A";
        }
        else if (singleCount >= noneCount && singleCount >= quadCount) {
            telemetry.addData("Single Ring Score  is " + singleCount + " out of last " + scansCount , " Zone B") ;
            zone ="B";
        }
        else {
            telemetry.addData("Quad Rings Score  is " + quadCount + " out of last " + scansCount , "Zone C") ;
            zone ="C";
        }
        telemetry.update();
        return zone;

    }

    public void straightenLeft(double timeOut)
    {
        double angle = -ultimateBot.getNavigation().leftAngle();
        ElapsedTime senseTime = new ElapsedTime();
        while(Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut)
        {
            angle = -ultimateBot.getNavigation().leftAngle();
        }
        if(angle > 50)
        {
            angle = 0;
        }
        telemetry.addData("Angle", angle);
        telemetry.update();

        if(Math.abs(angle) > 5)
        {
            encoderTurn(WHEEL_SPEED, angle, 5);
        }
    }


    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
    public void rampSpeedEncoderDrive(double speed, double inches, double timeoutS)
    {
        double setSpeed = speed;
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            ultimateBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);


            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            speed = 0.1;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                while((Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/4)
                        && Math.abs(speed) < Math.abs(setSpeed)))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while((Math.abs(speed) < Math.abs(setSpeed))
                        && Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while(Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                }
                while((Math.abs(ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) > Math.abs(newFrontLeftTarget/2)) && speed > 0.1)
                {
                    ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed -  0.01;
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderDrive

    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            ultimateBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }//end of encoderDrive

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);

            ultimateBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            ultimateBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            ultimateBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            ultimateBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            ultimateBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            ultimateBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (ultimateBot.getChassisAssembly().isBackLeftWheelBusy() && ultimateBot.getChassisAssembly().isBackRightWheelBusy() &&
                            ultimateBot.getChassisAssembly().isFrontLeftWheelBusy() && ultimateBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            ultimateBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            ultimateBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderTurn

    public void encoderSide(double speed, double inches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = chassis.getBackLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
            newBackRightTarget = chassis.getBackRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontLeftTarget = chassis.getFrontLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontRightTarget = chassis.getFrontRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);

            chassis.setBackLeftWheelTargetPosition(newBackLeftTarget);
            chassis.setBackRightWheelTargetPosition(newBackRightTarget);
            chassis.setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            chassis.setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            chassis.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            chassis.setBackLeftWheelPower(Math.abs(speed));
            chassis.setBackRightWheelPower(Math.abs(speed));
            chassis.setFrontLeftWheelPower(Math.abs(speed));
            chassis.setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (chassis.isBackLeftWheelBusy() && chassis.isBackRightWheelBusy() &&
                            chassis.isFrontLeftWheelBusy() && chassis.isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        chassis.getBackLeftWheelCurrentPosition(),
                        chassis.getBackRightWheelCurrentPosition(),
                        chassis.getFrontLeftWheelCurrentPosition(),
                        chassis.getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            chassis.stopMoving();

            // Turn off RUN_TO_POSITION
            chassis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }//end of encoderSide
}
