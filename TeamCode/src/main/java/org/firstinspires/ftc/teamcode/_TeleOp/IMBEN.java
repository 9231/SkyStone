/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

@TeleOp(name="Im Ben", group="Ben is Awesome too")

public class
IMBEN extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TopLeft = null;
    private DcMotor TopRight = null;
    private DcMotor BottomLeft = null;
    private DcMotor BottomRight = null;
    private DcMotor Waist = null;
    private DcMotor Shoulder = null;
    private Servo Xwrist = null;
    private Servo Ywrist = null;
    private Servo Grab = null;


    //bootleeeeeen
    private Boolean hasTopLeft =Boolean.FALSE;
    private Boolean hasTopRight =Boolean.FALSE;
    private Boolean hasBottomLeft =Boolean.FALSE;
    private Boolean hasBottomRight =Boolean.FALSE;
    private Boolean hasWaist =Boolean.FALSE;
    private Boolean hasShoulder =Boolean.FALSE;
    private Boolean hasXwrist =Boolean.FALSE;
    private Boolean hasYwrist =Boolean.FALSE;
    private Boolean hasGrab =Boolean.FALSE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        try{
            TopLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
            TopLeft.setDirection(DcMotor.Direction.FORWARD);
            hasTopLeft = Boolean.TRUE;
            telemetry.addData("TopLeft", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("TopLeft", "Failed");
        }


        try{
            TopRight  = hardwareMap.get(DcMotor.class, "FrontRight");
            hasTopRight = Boolean.TRUE;
            TopRight .setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("TopRight", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("TopRight", "Failed");
        }


        try{
            BottomRight  = hardwareMap.get(DcMotor.class, "BackRight");
            hasBottomRight = Boolean.TRUE;
            BottomRight.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("BottomRight", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("BottomRight", "Failed");
        }


        try{
            BottomLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
            hasBottomLeft = Boolean.TRUE;
            BottomLeft.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("BottomLeft", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("BottomLeft", "Failed");
        }


        try{
            Waist  = hardwareMap.get(DcMotor.class, "FrontLeft");
            Waist.setDirection(DcMotor.Direction.FORWARD);
            hasWaist = Boolean.TRUE;
            telemetry.addData("Waist", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("Waist", "Failed");
        }


        try{
            Shoulder  = hardwareMap.get(DcMotor.class, "FrontLeft");
            Shoulder.setDirection(DcMotor.Direction.FORWARD);
            hasShoulder = Boolean.TRUE;
            telemetry.addData("Shoulder", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("Shoulder", "Failed");
        }


        try{
            Xwrist  = hardwareMap.get(Servo.class, "FrontLeft");
            hasXwrist = Boolean.TRUE;
            telemetry.addData("Xwrist", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("Xwrist", "Failed");
        }


        try{
            Ywrist  = hardwareMap.get(Servo.class, "FrontLeft");
            hasYwrist = Boolean.TRUE;
            telemetry.addData("Ywrist", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("Ywrist", "Failed");
        }


        try{
            Grab  = hardwareMap.get(Servo.class, "FrontLeft");
            hasGrab = Boolean.TRUE;
            telemetry.addData("Grab", "Initialized");
        }
        catch (IllegalArgumentException iax)  {
            telemetry.addData("Grab", "Failed");
        }
        //ben is the best humanbeing on the planet
        //BET
        //also human being is not one word

        /* Initialize the hardware variables. Note that the strings used here as parameters
        to 'get' must correspond to the names assigned during the robot configuration
        step (using the FTC Robot Controller app on the phone). */
//        BottomLeft  = hardwareMap.get(DcMotor.class, "left_drive");
//        BottomRight = hardwareMap.get(DcMotor.class, "right_drive");
//        TopLeft = hardwareMap.get(DcMotor.class, "right_drive");
//        TopRight = hardwareMap.get(DcMotor.class, "right_drive");

        /* Most robots need the motor on one side to be reversed to drive forward
        Reverse the motor that runs backwards when connected directly to the battery */
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Bot", "Initialized");
        telemetry.addData("init", "Loop");
        telemetry.addData("10101001101","THE BOT IS WORKING 01101010101010101");
    }

    /*
     * Code to run REPEATEDLY after the driv
     * er hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double LeftRight = gamepad1.left_stick_x;
        double Forwardbackward = -gamepad1.left_stick_y;
        double Turning = gamepad1.right_stick_x;
        double LeftTrigger = gamepad1.left_trigger;
        double RightTrigger = gamepad1.right_trigger;
        double Angle;

//        double Xwrist = gamepad2.
//        double Ywrist = gamepad2.right_trigge
//        double Grab = gamepad2.left_stick
//        double Waist = gamepad2.left_stick
//        double Shoulder = gamepad2.left_stick

        //speed formula
        double Speed = (LeftTrigger * 0.4 + RightTrigger * 0.6) * Math.sqrt(2);



        double FrontRightPower = 0;
        double FrontLeftPower = 0;
        double BackRightPower = 0;
        double BackLeftPower = 0;

        double XwristPotition = 0;
        double YwristPotition = 0;
        double GrabPotition = 0;

        //this is getting rid of negative zero yay!
        if(LeftRight > -0.000001 && LeftRight < 0.000001){
            LeftRight = 0;
        }
        if(Forwardbackward > -0.000001 && Forwardbackward < 0.000001){
            Forwardbackward = 0;
        }
        if(Turning > -0.000001 && Turning < 0.000001){
            Turning = 0;
        }
        if(LeftTrigger > -0.000001 && LeftTrigger < 0.000001){
            LeftTrigger = 0;
        }
        if(RightTrigger > -0.000001 && RightTrigger < 0.000001){
            RightTrigger = 0;
        }
        if(LeftRight > -0.000001 && LeftRight < 0.000001){
            LeftRight = 0;

        }

        Angle = Math.toDegrees(Math.atan2(LeftRight,Forwardbackward));

        if(Angle < 0 ){
            Angle = Angle+360;
        }
        if (Math.abs(LeftRight)+Math.abs(Forwardbackward)< 0.8){
            FrontLeftPower =0;
            FrontRightPower =0;
            BackRightPower =0;
            BackLeftPower =0;
        }
        else {

            FrontLeftPower = getWheelPower(Angle, Speed, 45);
            FrontRightPower = getWheelPower(Angle, Speed, -45);
            BackLeftPower = getWheelPower(Angle, Speed, 135);
            BackRightPower = getWheelPower(Angle, Speed, -135);
        }

        // scale the direction by power buttons
        FrontLeftPower *= Speed;
        FrontRightPower *= Speed;
        BackLeftPower *= Speed;
        BackRightPower *= Speed;

        // scaling the turn by power buttons
        FrontLeftPower +=  -Turning * Speed;
        FrontRightPower += -Turning * Speed;
        BackLeftPower += -Turning * Speed;
        BackRightPower +=  -Turning * Speed;

        ArrayList<Double> Box = new ArrayList<>();
        Box.add(Math.abs(FrontLeftPower));
        Box.add(Math.abs(FrontRightPower));
        Box.add(Math.abs(BackLeftPower));
        Box.add(Math.abs(BackRightPower));
        double biggest = Collections.max(Box);

        if(biggest > 1) {
            FrontRightPower /= biggest;
            FrontLeftPower /= biggest;
            BackRightPower /= biggest;
            BackLeftPower /= biggest;
        }


        if(hasTopLeft) {
            TopLeft.setPower(FrontLeftPower);
        }
        if(hasTopRight) {
            TopRight.setPower(FrontRightPower);
        }
        if(hasBottomLeft) {
            BottomLeft.setPower(BackLeftPower);
        }
        if(hasBottomRight) {
            BottomRight.setPower(BackRightPower);
        }



        telemetry.addData("left_stick_x", LeftRight);
        telemetry.addData("left_stick_y", Forwardbackward);
        telemetry.addData("right_stick_x", Turning);
        telemetry.addData("left_trigger", LeftTrigger);
        telemetry.addData("right_trigger", RightTrigger);
        telemetry.addData("LeftStick_Degrees", Angle);
        telemetry.addData("    MotorPower","Left Front(%.2f), Right Front (%.2f)", FrontLeftPower, FrontRightPower);
        telemetry.addData("MotorPower","Left Back(%.2f), Right Back (%.2f)", BackLeftPower, BackRightPower);

    }

    @Override
    public void stop() {
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }

    /**
     * get wheel power turns polar coordinates into various wheel powers
     * @param angle - the direction into various wheel powers
     * @param speed - how fast you want to get there
     * @param Motoradjustment - angle that the motor is at
     * @return - the motor power for that motorAdjustment
     */
    private double getWheelPower(double angle, double speed, double Motoradjustment){
        //x and y
        double newMotorX = speed * Math.cos(Math.toRadians(Motoradjustment - angle));

        return newMotorX;
    }
}