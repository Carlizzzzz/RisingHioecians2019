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

@TeleOp(name = "God", group = "Iterative Opmode")
public class God extends setting {
    // Declare OpMode members.


    @Override
    public void init() {
        telemetry.addData("Status", "Start initialize");


        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");


        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);


        UD_init();
        IO_init();
        hang_init();
        hold_init();


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

        telemetry.update();


        if (UD) {
            if (gamepad1.left_bumper) {  //Up
//            UpDown.setTargetPosition(-5400);
                UpDown.setPower(1);
            } else if (gamepad1.right_bumper) { //down
//            UpDown.setTargetPosition(-100);
                UpDown.setPower(-1);
            } else {
                UpDown.setPower(0);
            }
            telemetry.addData("updown Value", UpDown.getCurrentPosition());
        }


        if (IO) {
            if (gamepad1.left_trigger > 0.5) {  //in
                inout.setPower(1);
            } else if (gamepad1.right_trigger > 0.5) { //out
                inout.setPower(-1);
            } else {
                inout.setPower(0);
            }
            telemetry.addData("inout Value", inout.getCurrentPosition());
        }


        if (hold) {
            if (gamepad1.dpad_up) {
                holder.setTargetPosition(-550);
                holder.setPower(0.7);
            } else if (gamepad1.dpad_left) {
                if (holder.getCurrentPosition() == -330) {
                    holder.setPower(0);
                } else {
                    holder.setTargetPosition(-330);
                    holder.setPower(0.7);
                }
            } else if (gamepad1.dpad_down) {

                holder.setTargetPosition(10);
                holder.setPower(-0.7);
            } else {
                holder.setPower(0);
            }
            telemetry.addData("holder Value", holder.getCurrentPosition());
        }

        if (hang) {
            if (gamepad1.dpad_right) {
                if (hanger.getPower() == 0) {
                    hanger.setPower(1);
                } else if (hanger.getPower() == 1) {
                    hanger.setPower(0);
                }
            } else {
                hanger.setPower(0);
            }
            telemetry.addData("hanger Value", hanger.getCurrentPosition());
        }

        if (spin) {
            if (gamepad1.b) {
                spinner.setPower(1);
            } else if (gamepad1.a) {
                spinner.setPower(-1);
            } else {
                spinner.setPower(0);
            }
        }


        telemetry.addData("LF Value", LF.getCurrentPosition());
        telemetry.addData("LB Value", LB.getCurrentPosition());
        telemetry.addData("RF Value", RF.getCurrentPosition());
        telemetry.addData("RB Value", RB.getCurrentPosition());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}

