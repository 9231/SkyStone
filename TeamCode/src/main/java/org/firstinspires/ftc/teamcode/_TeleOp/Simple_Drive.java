/* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Gonna push this file.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="Simple_Drive", group="!Savvy is Awesome with a capital A")  // @Autonomous(...) is the other common choice
//@Disabled
public class Simple_Drive extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    // savvy snatcher
    DcMotor motorWaist;
    DcMotor motorShoulder;
    Servo servoElbow;
    private Servo Xwrist = null;
    private Servo Grab = null;
    private Servo servoFlip3 = null;
    private Servo servoFlip4 = null;
//    private Servo Ywrist = null;

    boolean bDebugFrontRight = false;
    boolean bDebugFrontLeft = false;
    boolean bDebugBackRight = false;
    boolean bDebugBackLeft = false;
    //snatcher
    boolean bDebugWaist = false;
    boolean bDebugShoulder = false;
    boolean bDebugElbow = false;
    private Boolean hasXwrist =Boolean.FALSE;
//    private Boolean hasYwrist =Boolean.FALSE;
    private Boolean hasGrab =Boolean.FALSE;
    private Boolean bDebugFlip3 =Boolean.FALSE;
    private Boolean bDebugFlip4 =Boolean.FALSE;


    // savvy snatcher
    float waistRotation = 0;
    float shoulderRotation = 0;
    double save_dpad_XY = 0;


    boolean dontTurn = false;

    /**
     * Constructor
     */
    public Simple_Drive() {
        telemetry.addData("BOT", String.format("ON"));

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */

        /*
         * For this test, we assume the following,
         *   There are four motors
         *   "FrontLeft" and "BackLeft" are front and back left wheels
         *   "FrontRight" and "BackRight" are front and back right wheels
         */
        /*
            debugging, basically telling the phone "if this motor is working then continue, but if
            there's a problem with anything (hardware, software, whatever isn't making
            the code work atm) then do bDebug..."
        */

        // chassis motors
        try {
            motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        } catch (IllegalArgumentException iax) {
            bDebugFrontRight = true;
        }
        try {
            motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (IllegalArgumentException iax) {
            bDebugFrontLeft = true;
        }
        try {
            motorBackRight = hardwareMap.dcMotor.get("BackRight");
        } catch (IllegalArgumentException iax) {
            bDebugBackRight = true;
        }
        try {
            motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (IllegalArgumentException iax) {
            bDebugBackLeft = true;
        }

        // savvy snatcher
        try{
            motorWaist = hardwareMap.dcMotor.get("Waist1");
            motorWaist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorWaist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (IllegalArgumentException iax) {
            bDebugWaist = true;
        }
        try {
            motorShoulder = hardwareMap.dcMotor.get("Shoulder0");
            motorShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (IllegalArgumentException iax) {
            bDebugShoulder = true;
        }
        try{
            servoElbow = hardwareMap.servo.get("Elbow");
        } catch (IllegalArgumentException iax) {
            bDebugElbow = true;
        }
        try{
            Grab  = hardwareMap.get(Servo.class, "Grab");
            hasGrab = Boolean.TRUE;
            telemetry.addData("Grab", "Initialized");
        } catch (IllegalArgumentException iax)  {
            telemetry.addData("Grab", "Failed");
        }
        try{
            Xwrist  = hardwareMap.get(Servo.class, "Xwrist");
            hasXwrist = Boolean.TRUE;
            telemetry.addData("Xwrist", "Initialized");
        } catch (IllegalArgumentException iax)  {
            telemetry.addData("Xwrist", "Failed");
        }
        try{
            servoFlip3 = hardwareMap.get(Servo.class, "Flip3");
            bDebugFlip3 = Boolean.TRUE;
            telemetry.addData("Flip3", "Successful");
        } catch (IllegalArgumentException iax){
            telemetry.addData("Flip3", "FAILED");
        }
        try{
            servoFlip4 = hardwareMap.get(Servo.class, "Flip4");
            bDebugFlip4 = Boolean.TRUE;
            telemetry.addData("Flip4", "Successful");
        } catch (IllegalArgumentException iax){
            telemetry.addData("Flip4", "FAILED");
        }
//        try{
//            Ywrist  = hardwareMap.get(Servo.class, "Ywrist");
//            hasYwrist = Boolean.TRUE;
//            telemetry.addData("Ywrist", "Initialized");
//        } catch (IllegalArgumentException iax) {
//            telemetry.addData("Ywrist", "Failed");
//        }

    }

    @Override
    public void loop() {
        /* left joystick is for rotating the waist (x)
         * also for rotating the wrist (y)
         *
         * right joystick is for lifting the snatcher with shoulder motor (y)
         * also for rotating the elbow (x)
         *
         * try to make it so if the shoulder goes up the elbow goes the opposite amount
         * (to remain level to the ground)
         * potentially make a button that if pressed will allow manual adjustment
         */

        // tank drive
        // if y is - then joystick is pushed all the way forward
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float r1 = gamepad1.right_stick_x;
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        float x3 = gamepad2.left_stick_x; //elbow
        float y3 = -gamepad2.left_stick_y; //wrist
        float x4 = gamepad2.right_stick_x; //waist
        float y4 = -gamepad2.right_stick_y; //shoulder
        boolean ControllerXwristL = gamepad2.dpad_left; //wrist
        boolean ControllerXwristR = gamepad2.dpad_right; //wrist
        boolean ControllerGrab = gamepad2.right_bumper; //finger
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;


        // lt = half speed
        float lt = gamepad1.left_trigger;
        // rt 2x speed
        float rt = gamepad1.right_trigger;

        // lt/rt = half speed
        float lt2 = gamepad2.left_trigger;
        float rt2 = gamepad2.right_trigger;

        // clip the left/right values so that the values never exceed +/- 1
        x1 = Range.clip(x1, -1, 1);
        y1 = Range.clip(y1, -1, 1);
        r1 = Range.clip(r1, -1, 1);
        // snazzy snatcher controls
        x3 = Range.clip(x3, -1, 1);
        y3 = Range.clip(y3, -1, 1);
//        x4 = Range.clip(x4, -1, 1);
//        y4 = Range.clip(y4, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds
        x1 = (float)scaleInput(x1);
        y1 = (float)scaleInput(y1);
        r1 = (float)scaleInput(r1);
        // snazzy snatcher controls
        x3 = (float)scaleInput(x3);
//        y3 = (float)scaleInput(y3);
//        x4 = (float)scaleInput(x4);
//        y4 = (float)scaleInput(y4);

        // power set to the motors
        float FrontRight = (y1-x1)-r1;
        float BackRight = (y1+x1)-r1;
        float FrontLeft = (y1+x1+r1);
        float BackLeft= (y1-x1)+r1;
//        // power set to the snazzy snatcher motors ~ /20 for testing purposes
//        float elbowRotationIncrement = x3/20;
//        elbowRotation += elbowRotationIncrement;


        if(Math.abs(FrontRight) > 1 && Math.abs(FrontRight) >= Math.abs(BackRight) && Math.abs(FrontRight) >= Math.abs(FrontLeft) && Math.abs(FrontRight) >= Math.abs(BackLeft)){
            BackRight = BackRight / Math.abs(FrontRight);
            FrontLeft = FrontLeft / Math.abs(FrontRight);
            BackLeft = BackLeft / Math.abs(FrontRight);
            FrontRight = FrontRight / Math.abs(FrontRight);
        } else if(Math.abs(BackRight) > 1 && Math.abs(BackRight) >= Math.abs(FrontRight) && Math.abs(BackRight) >= Math.abs(FrontLeft) && Math.abs(BackRight) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(BackRight);
            FrontLeft = FrontLeft / Math.abs(BackRight);
            BackLeft = BackLeft / Math.abs(BackRight);
            BackRight = BackRight / Math.abs(BackRight);
        } else if(Math.abs(FrontLeft) > 1 && Math.abs(FrontLeft) >= Math.abs(FrontRight) && Math.abs(FrontLeft) >= Math.abs(BackRight) && Math.abs(FrontLeft) >= Math.abs(BackLeft)){
            FrontRight = FrontRight / Math.abs(FrontLeft);
            BackRight = BackRight / Math.abs(FrontLeft);
            BackLeft = BackLeft / Math.abs(FrontLeft);
            FrontLeft = FrontLeft / Math.abs(FrontLeft);
        } else if(Math.abs(BackLeft) > 1 && Math.abs(BackLeft) >= Math.abs(FrontRight) && Math.abs(BackLeft) >= Math.abs(BackRight) && Math.abs(BackLeft) >= Math.abs(FrontLeft)){
            FrontRight = FrontRight / Math.abs(BackLeft);
            BackRight = BackRight / Math.abs(BackLeft);
            FrontLeft = FrontLeft / Math.abs(BackLeft);
            BackLeft = BackLeft / Math.abs(BackLeft);
        }


        //bruh when you don't not want the robot to turn
        dontTurn = false;

		// value of motors/servos
        FrontRight /= 2;
        BackRight /= 2;
        FrontLeft /= 2;
        BackLeft /= 2;
        FrontRight *= (1-(lt/2));
        BackRight *= (1-(lt/2));
        FrontLeft *= (1-(lt/2));
        BackLeft *= (1-(lt/2));
        FrontRight *= (1+rt);
        BackRight *= (1+rt);
        FrontLeft *= (1+rt);
        BackLeft *= (1+rt);

        double GGrab = ControllerGrab ? 0.96 : 0.69 ;
        int waistTarget = (int)(x4 * 600);
        int shoulderTarget = (int)(y4 * 2000);
        waistTarget = waistTarget < -150 ? -150: waistTarget > 550 ? 550: waistTarget;
        shoulderTarget = shoulderTarget < -200 ? -200: shoulderTarget > 1530 ? 1530: shoulderTarget;

//         double wristValue = Xwrist.getPosition();

        // if the motors didn't fail but their supposed to be zero then set power to zero
        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebugFrontRight || !bDebugBackRight || !bDebugFrontLeft || !bDebugBackLeft || !bDebugWaist || !bDebugShoulder || !bDebugElbow || !hasGrab || !hasXwrist) {
            if(FrontRight == 0){
                motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else{
                motorFrontRight.setPower(FrontRight);
            }
            if(BackRight == 0){
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else{
                motorBackRight.setPower(BackRight);
            }
            if(FrontLeft == 0){
                motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else{
                motorFrontLeft.setPower(FrontLeft);
            }
            if(BackLeft == 0){
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else{
                motorBackLeft.setPower(BackLeft);
            }
            //snazzy snatcher
            if(waistTarget == 0){
                motorWaist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motorWaist.setTargetPosition(waistTarget);
                motorWaist.setPower(0.05);
                motorWaist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
			if(shoulderTarget == 0){
                motorShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			} else {
                motorShoulder.setTargetPosition(shoulderTarget);
                motorShoulder.setPower(0.08);
                motorShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            servoElbow.setPosition(y3 + 0.7086);

			if (hasGrab) {
                Grab.setPosition(GGrab);
            }
            if (ControllerXwristR) {
                save_dpad_XY = 0.8;
                Xwrist.setPosition(save_dpad_XY);
            } else if (ControllerXwristL) {
                save_dpad_XY = 0.4826;
                Xwrist.setPosition(save_dpad_XY);
            }
            if (a){
                motorWaist.setTargetPosition(0);
                Xwrist.setPosition(0.8);
                motorShoulder.setTargetPosition(1000);
                servoElbow.setPosition(0.7086);
            }
            if (b){
                motorWaist.setTargetPosition(0);
                Xwrist.setPosition(0.8); 
                motorShoulder.setTargetPosition(-20);
                servoElbow.setPosition(0.7086);
            }
//            } else if (hasXwrist) {
//                Xwrist.setPosition(x3 + 0.4826);
//            } else if (x3 == 0) {
//                Xwrist.setPosition(wristValue);
//            }

      /*
      if(r != 0) {
         x = 0;
         y = 0;
         motorFrontRight.setPower(-r);
         motorBackRight.setPower(-r);
         motorFrontLeft.setPower(r);
         motorBackLeft.setPower(r);
      }
       */
        }

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */
        telemetry.addData("T-t-testing in p-p-progress... UwU (© w ©)", "U w U");

        /*
         * Checks for each wheel's power, and if the wheel setup ran into an error,
         * will return 'not working' instead of a power.
         */
        if(!bDebugFrontRight){
            telemetry.addData("front right pwr", String.format("%.2f", FrontRight));
        } else{
            telemetry.addData("front right pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back right pwr", String.format("%.2f", BackRight));
        } else{
            telemetry.addData("back right pwr", String.format("not working"));
        }
        if(!bDebugFrontLeft){
            telemetry.addData("front left pwr", String.format("%.2f", FrontLeft));
        } else{
            telemetry.addData("front left pwr", String.format("not working"));
        }
        if(!bDebugBackRight){
            telemetry.addData("back left pwr", String.format("%.2f", BackLeft));
        } else{
            telemetry.addData("back left pwr", String.format("not working"));
        }
        // gamepad1 controls that control speed
        telemetry.addData("left trigger on gamepad 1", String.format("%.2f",lt));
        telemetry.addData("right trigger on gamepad 1", String.format("%.2f",rt));
        telemetry.addData("gamepad1", gamepad1);


        // testing
        telemetry.addData("waist", motorWaist.getCurrentPosition());
        telemetry.addData("shoulder", motorShoulder.getCurrentPosition());

        telemetry.addData("waist go to", waistTarget);
        telemetry.addData("shoulder go to", shoulderTarget);
        telemetry.addData("finger", ControllerGrab);
        telemetry.addData("Xwrist", y3);
        telemetry.addData("Elbow", x3);


        // snazzy snatcher motors
        if(!bDebugWaist){
            telemetry.addData("waist motor (1)", String.format("%.2f", waistRotation));
        } else{
            telemetry.addData("waist motor (1)", String.format("malfunctioning"));
        }
        if(!bDebugShoulder){
            telemetry.addData("shoulder motor (0)", String.format("%.2f", shoulderRotation));
        } else{
            telemetry.addData("shoulder motor (0)", String.format("not working"));
        }
        if(!bDebugElbow){
            telemetry.addData("elbow servo", String.format("%.2f",x3));
        } else{
            telemetry.addData("elbow servo", String.format("BLEEP BLOOP how about NOPE ha noob"));
        }

        // gamepad2 controls that control speed
        telemetry.addData("left trigger on gamepad 2", String.format("%.2f",lt2));
        telemetry.addData("right trigger on gamepad 2", String.format("%.2f",rt2));
        telemetry.addData("gamepad2", gamepad2);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        return dVal*dVal*dVal;    // maps {-1,1} -> {-1,1}
    }

}

//Bruh
//big-gy b-b-brains ~~~~ UwU  (˚ ∆ ˚)  (© o ©)  (• w •)
