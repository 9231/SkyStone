package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._Libs.Xdrive;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoFoundationMover", group="Xdrive")
//@Disabled
public class AutoFoundationMover extends LinearOpMode {

    Xdrive robot   = new Xdrive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //declaring servos
    private Servo leftToe;
    private Servo rightToe;

    //
    private Boolean hasLeftToe = false;
    private Boolean hasRightToe = false;

    static final double     FORWARD_SPEED = 0.4;
    static final double     BACKWARD_SPEED = -0.4;
//    static final double     TURN_SPEED    = 0.4;

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //track if servos are connected
        try {
            leftToe = hardwareMap.get(Servo.class, "leftToe");
            telemetry.addData("servo", "Left toe initialized");
            hasLeftToe = true;

            //make left toe begin in upwards position
            leftToe.setPosition(0.7);
        }

        catch (IllegalArgumentException iax) {
            telemetry.addData("servo", "Servo isn't working you diddly dumbdumb ding fling");
        }

        //right toe connection
        try {
            rightToe = hardwareMap.get(Servo.class, "rightToe");
            telemetry.addData("servo", "Right toe is initialized");
            hasRightToe = true;

            //make right toe begin in upwards position
            rightToe.setPosition(0);
        }
        catch (IllegalArgumentException iax) {
            telemetry.addData("servo", "Servo isn't working you diddly dumbdumb ding fling");
        }


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        waitForStart();

        runtime.reset();

        // Move left
        robot.FrontLeft.setPower(BACKWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.BackLeft.setPower(FORWARD_SPEED);
        robot.BackRight.setPower(BACKWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 2.2)) {
            telemetry.addData("Direction", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Stop
        runtime.reset();

        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
        robot.BackLeft.setPower(0);
        robot.BackRight.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Direction", "Stopped: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Move forward
        runtime.reset();

        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(FORWARD_SPEED);
        robot.BackLeft.setPower(FORWARD_SPEED);
        robot.BackRight.setPower(FORWARD_SPEED);


        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Direction", "Left: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();

        // Stop all motors
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
        robot.BackLeft.setPower(0);
        robot.BackRight.setPower(0);


        if (hasRightToe) {
            rightToe.setPosition(0.7);
        }

        //Move toes to position
        if (hasLeftToe) {
            leftToe.setPosition(0);
        }

        //Stop
        runtime.reset();

        //keep motors stopped
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
        robot.BackLeft.setPower(0);
        robot.BackRight.setPower(0);

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Direction", "Stopped", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();

        // Move right
        robot.FrontLeft.setPower(FORWARD_SPEED);
        robot.FrontRight.setPower(BACKWARD_SPEED);
        robot.BackLeft.setPower(BACKWARD_SPEED);
        robot.BackRight.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 1.9)) {
            telemetry.addData("Direction", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
