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

import java.util.List;




@Autonomous(name = "GoalAutoBlue", group = "Qualifier")
public class GoalAutoBlue extends LinearOpMode {
   
    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 0.5;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 50;
    final double COUNTS_PER_DEGREE = 8.5;

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
    TFObjectDetector tfod = null;


    //Time
    ElapsedTime runtime = new ElapsedTime();
    int numRings = 0;

    @Override public void runOpMode() {
        
        //Intialize Robot
        ultimateBot.initRobot(hardwareMap);
        vcortex = ultimateBot.getVisualCortex();
        nav = ultimateBot.getNavigation();
        chassis = ultimateBot.getChassisAssembly();;
        allTrackables = vcortex.getTrackables();
        tfod =  vcortex.getTfod();
        //Wait for Start
        telemetry.addData("Waiting for start", "");
        VuforiaTrackable blueTower = allTrackables.get(0);
        int[] stackArray = {0, 0, 0};
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Scanning stack ", "..Waiting for Start");
            telemetry.update();
            int stack = vcortex.checkStarterStack(3);
            if(stack==0)
            {
                stackArray[0] = stackArray[0] + 1;
            }
            else if(stack==1)
            {
                stackArray[1] = stackArray[1] + 1;
            }
            else if(stack==4)
            {
                stackArray[2] = stackArray[2] + 1;
            }
        }


        //waitForStart();

        /**
         * DURING RUNTIME
         */

        telemetry.addData("Starting Autonomous ", "");
        String recognizedStack =   getStackScanResult(stackArray);
        sleep(2000);


        //preparation();
        //alignWithTarget(blueTower);
    }

    public String  getStackScanResult(int stack[])
    {
        int highestValue;
        int index = 0;
        int i = 0;
        highestValue = stack[0];
        int totalTries = 0;
        String stackScanned = "NONE";
        while(i<3)
        {
            totalTries = totalTries + stack[i];
            if(highestValue < stack[i])
            {
                highestValue = stack[i];
                index = i;
            }
            i++;
        }
        telemetry.addData("Score  is " + highestValue + "out of " + totalTries , "") ;

        if(index==0)
        {
            telemetry.addData("Stack has  ", "None");
            stackScanned ="NONE";
        }
        else if(index==1)
        {
            telemetry.addData("Stack has  ", "One");
            stackScanned ="ONE";
        }
        else
        {
            telemetry.addData("Stack has  ", "Four");
            stackScanned ="QUAD";
        }
        telemetry.update();
        return stackScanned;

    }
    public void preparation()
    {
        runtime.reset();

        //Move forward in preperation to Scan for skystone target
        encoderDrive(WHEEL_SPEED, 15, 10);
        chassis.stopMoving();

        encoderSide(WHEEL_SPEED, -8, 10);


    }




    public void alignWithTarget(VuforiaTrackable target)
    {
        straightenLeft(1);
        //Get the new location of the target and ensure that the target is still visible
        OpenGLMatrix lastLocation = vcortex.getCurrentLocationFromTarget(target, 5);

        if (lastLocation !=null) {
            //Find the position of the target
            VectorF translation = lastLocation.getTranslation();
            double currentPos = nav.rightDistance();

            double targetShift = translation.get(1)/mmPerInch;

            //Target Position
            double ringPos = currentPos - targetShift + 14;//plus 4 because of the the distance betweeen the camera and the sensors

            numRings = (int) (Math.ceil(ringPos / 8));

            //Print the Result
            telemetry.addData("currentPos", currentPos);
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            telemetry.addData("Block Position", ringPos);
            telemetry.addData("Block #", numRings);
            telemetry.addData("Shift" , Math.abs(targetShift + 8));
            telemetry.update();
            sleep(250);

            //Shift the Robot so that the arm is Centered
            if(Math.abs(targetShift + 8) > 2)
            {
                encoderSide(WHEEL_SPEED, targetShift + 8, 5);
                chassis.stopMoving();
            }
        }
        else
        {
            telemetry.addData("target no longer visible!", "");
            telemetry.update();
            sleep(4000);
        }



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
    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            chassis.changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = chassis.getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = chassis.getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = chassis.getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = chassis.getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



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
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
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

    }//end of encoderDrive

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


    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = chassis.getBackLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = chassis.getBackRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = chassis.getFrontLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = chassis.getFrontRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);

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
    }//end of encoderTurn
}
