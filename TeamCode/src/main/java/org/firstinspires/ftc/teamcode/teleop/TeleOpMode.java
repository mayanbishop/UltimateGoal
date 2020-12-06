package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.UltimateBot;

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
    final double COUNTS_PER_DEGREE = 9;
    double shooterAngle = 0.85;

    boolean isIntaking = false;
    boolean intakePressed = false;


    //Creating a Rover robot object
    UltimateBot ultimateBot = new UltimateBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ultimateBot.initRobot(hardwareMap);
        ultimateBot.getShooterAssembly().returnPusher();
        ultimateBot.getShooterAssembly().highGoalAng();
        ultimateBot.getWobbleAssembly().openGripper();

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

            double armMovement = gamepad1.right_stick_y;

            boolean intake = gamepad1.x;
            boolean shoot = gamepad1 .b;

            boolean gripClose = gamepad1.a;
            boolean gripOpen = gamepad1.y;

            boolean highGoal = gamepad1.dpad_up;
            boolean powerShot = gamepad1.dpad_down;

            //Gamepad 2 Controls
            boolean midGoal = gamepad2.a;
            boolean outake = gamepad2.x;
            boolean stopOutake = gamepad2.b;

            boolean shooterUp = gamepad2.dpad_up;
            boolean shooterDown = gamepad2.dpad_down;

            //Movement
            if (drive < 0) {
                ultimateBot.getChassisAssembly().moveForward(Math.abs(drive));
            }
            else if (drive > 0){
                ultimateBot.getChassisAssembly().moveBackwards(Math.abs(drive));
            }
            //turn right
            else if (turn > 0) {
                ultimateBot.getChassisAssembly().turnRight(Math.abs(turn));
            }
            //turn left
            else if (turn < 0) {
                ultimateBot.getChassisAssembly().turnLeft(Math.abs(turn));
            }
            //side right
            else if (sideRight > 0) {
                ultimateBot.getChassisAssembly().moveRight(sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                ultimateBot.getChassisAssembly().moveLeft(sideLeft);
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
            else
            {
                ultimateBot.getChassisAssembly().stopMoving();
            }

            if(intake == false && intakePressed == true)
            {
                intakePressed = false;
            }
            if(intake == true && isIntaking == false && intakePressed == false)
            {
                ultimateBot.getIntakeAssembly().intake();
                isIntaking = true;
                intakePressed = true;
            }
            if(intake == true && isIntaking == true && intakePressed == false)
            {
                ultimateBot.getIntakeAssembly().stopIntake();
                isIntaking = false;
                intakePressed = true;
            }

            if(shoot == true)
            {
                ultimateBot.getIntakeAssembly().stopIntake();
                ultimateBot.getShooterAssembly().shoot();

                while(opModeIsActive() && ultimateBot.getRobotHardware().topTouch.isPressed() == false)
                {
                    ultimateBot.getShooterAssembly().moveLift(1.0);
                }
                ultimateBot.getShooterAssembly().stopLift();
                ultimateBot.getShooterAssembly().openDoor();
                sleep(500);

                for(int i = 0; i < 3; i++)
                {
                    ultimateBot.getShooterAssembly().pushRing();
                    sleep(500);
                    ultimateBot.getShooterAssembly().returnPusher();
                    sleep(500);
                }

                while(opModeIsActive() && ultimateBot.getRobotHardware().bottomTouch.isPressed() == false)
                {
                    ultimateBot.getShooterAssembly().moveLift(-0.5);
                }
                ultimateBot.getShooterAssembly().stopLift();
                ultimateBot.getShooterAssembly().closeDoor();

                ultimateBot.getShooterAssembly().stopShoot();
            }

            if(powerShot == true)
            {
                ultimateBot.getShooterAssembly().powerShotAng();
            }

            if(highGoal == true)
            {
                ultimateBot.getShooterAssembly().highGoalAng();
            }

            if(gripOpen == true)
            {
                ultimateBot.getWobbleAssembly().openGripper();
                sleep(500);

                while(opModeIsActive() &&  ultimateBot.getRobotHardware().armReturn.isPressed() == false)
                {
                    ultimateBot.getWobbleAssembly().moveArm(1.0);
                }
                ultimateBot.getWobbleAssembly().stopArm();
            }

            if(gripClose == true)
            {
                ultimateBot.getWobbleAssembly().closeGripper();
            }

            if(armMovement > 0 &&  ultimateBot.getRobotHardware().grabTouch.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(-1.0);
            }
            else if(armMovement < 0 &&  ultimateBot.getRobotHardware().armReturn.isPressed() == false)
            {
                ultimateBot.getWobbleAssembly().moveArm(0.5);
            }
            else
            {
                ultimateBot.getWobbleAssembly().stopArm();
            }

            //Gamepad 2

            if(midGoal == true)
            {
                ultimateBot.getShooterAssembly().midGoalAng();
            }

            if(outake == true)
            {
                ultimateBot.getIntakeAssembly().outake();
            }

            if(stopOutake == true)
            {
                ultimateBot.getIntakeAssembly().stopIntake();
            }

            //up pos = 0.35, down pos = 0.9
            if(shooterUp == true)
            {
                ultimateBot.getShooterAssembly().changeShooterAng(shooterAngle);
                sleep(100);
                shooterAngle = shooterAngle - 0.01;
                telemetry.addData("Shooter Angle", shooterAngle);
                telemetry.update();
            }

            if(shooterDown == true)
            {
                ultimateBot.getShooterAssembly().changeShooterAng(shooterAngle);
                sleep(100);
                shooterAngle = shooterAngle + 0.01;
                telemetry.addData("Shooter Angle", shooterAngle);
                telemetry.update();
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

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = ultimateBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = ultimateBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = ultimateBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = ultimateBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);

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
}