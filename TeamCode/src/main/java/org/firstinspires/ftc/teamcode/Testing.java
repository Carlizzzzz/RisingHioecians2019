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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Testing", group = "Iterative Opmode")
public class Testing extends OpMode {
    // Declare OpMode members.
    BNO055IMU gyro;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF, LB, RF, RB, motor1, motor2 = null;
    private CRServo roll, pull = null;
    private Servo mark,upServo1, upServo2 = null;

    boolean constant_up = false, max_up = false;
    int upServoState = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        upServo1 = hardwareMap.get(Servo.class, "upServo1");
        upServo2 = hardwareMap.get(Servo.class, "upServo2");
        roll = hardwareMap.get(CRServo.class, "roll");
        pull = hardwareMap.get(CRServo.class, "pull");
        mark = hardwareMap.get(Servo.class, "mark");
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);


        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        telemetry.addData("Status", "Initialized");


    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LFPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double LBPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double RFPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double RBPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

        LFPower = Range.clip(LFPower, -0.7, 0.7);
        RFPower = Range.clip(RFPower, -0.7, 0.7);
        LBPower = Range.clip(LBPower, -0.7, 0.7);
        RBPower = Range.clip(RBPower, -0.7, 0.7);

        LF.setPower(LFPower);
        RF.setPower(RFPower);
        LB.setPower(LBPower);
        RB.setPower(RBPower);

        if (gamepad1.right_bumper) {
            motor1.setPower(.55);
            motor2.setPower(.55);
        }else if (gamepad1.right_trigger>0.5) {
            motor1.setPower(-.55);
            motor2.setPower(-.55);
        } else {
        motor1.setPower(0.1);
        motor2.setPower(0.1);
    }




        if (gamepad1.b) {
            //pull
            pull.setPower(-1);
        } else {
            pull.setPower(0);
        }

        if(gamepad1.y){
            if(upServoState==0) {
                upServo1.setPosition(0);
                upServo2.setPosition(1);
                upServoState=1;
            }else if(upServoState==1){
                upServo1.setPosition(0.5);
                upServo2.setPosition(0.5);
                upServoState=0;
            }
        }
//
//        if (gamepad1.y) {// extend arm perpendicular
//           constant_up = !constant_up;
//        }
//        if (constant_up) {
//            motor1.setPower(0.5);
//            motor2.setPower(0.5);
//        }
//
//        if (gamepad1.b) {   //turn the arm up in a max speed
//            max_up = !max_up;
//        }
//        if (max_up) {
//            motor1.setPower(1);
//            motor2.setPower(1);
//        }


        if (gamepad1.left_bumper) {
            roll.setPower(1);
        } else if (gamepad1.left_trigger>0.5) {
            roll.setPower(-1);
        } else {
            roll.setPower(0);
        }

        telemetry.update();
        telemetry.addData("LF Value", LF.getCurrentPosition());
        telemetry.addData("LB Value", LB.getCurrentPosition());
        telemetry.addData("RF Value", RF.getCurrentPosition());
        telemetry.addData("RB Value", RB.getCurrentPosition());
        telemetry.addData("motor1 position", motor1.getCurrentPosition());
        telemetry.addData("motor2 position", motor2.getCurrentPosition());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
