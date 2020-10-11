package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode {
    private double   WHEEL_SPEED = 1.0;
    private static double INTAKE_SPEED = 1.0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    //Creating a Rover robot object
    UltimateBot ultimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ultimateBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive())
        {
            //Gamepad 1 Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;

            boolean intake = gamepad1.x;
            boolean outake = gamepad2.a; //changed to gamepad 2


            //Movement
            if (drive > 0) {
                ultimateBot.getChassisAssembly().moveForward(-WHEEL_SPEED * drive);
            }
            //backwards
            else if (drive < 0) {
                ultimateBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * drive);
            }
            //turn right
            else if (turn > 0) {
                ultimateBot.getChassisAssembly().turnRight(WHEEL_SPEED * turn);
            }
            //turn left
            else if (turn < 0) {
                ultimateBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -turn);
            }
            //side right
            else if (sideRight > 0) {
                ultimateBot.getChassisAssembly().moveRight(WHEEL_SPEED * sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                ultimateBot.getChassisAssembly().moveLeft(WHEEL_SPEED * sideLeft);
            }
         /*   else if(diagonalFrontRight > 0)
            {
                ultimateBot.getChassisAssembly().diagonalForwardRight(WHEEL_SPEED);
            }
            else if(diagonalFrontLeft < 0)
            {
                ultimateBot.getChassisAssembly().diagonalForwardLeft(WHEEL_SPEED);
            }
            else if(diagonalBackRight > 0)
            {
                ultimateBot.getChassisAssembly().diagonalBackwardsRight(WHEEL_SPEED);
            }
            else if(diagonalBackLeft < 0)
            {
                ultimateBot.getChassisAssembly().diagonalBackwardsLeft(WHEEL_SPEED);
            }
          */
            //stop moving
            else {
                ultimateBot.getChassisAssembly().stopMoving();
            }
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
}