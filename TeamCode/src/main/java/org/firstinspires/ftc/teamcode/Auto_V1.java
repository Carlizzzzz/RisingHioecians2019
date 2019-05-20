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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "autoV1", group = "Linear Opmode")
//@Disabled
public class Auto_V1 extends LinearOpMode {

    BNO055IMU gyro;
    int state = 0;
    int goldPosition = -1;
    private ElapsedTime runtime = new ElapsedTime();
    private AutoWheelBase autoWheelBase1;
    private DcMotor LF, LB, RF, RB, motor1, motor2, motor3 = null;
    Servo upServo1, upServo2, mark;
    CRServo roll, pull;
    double timeCounter = 0;

    void openloopLift(double power) {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
    }

    void posCtrlLift(int pos) {
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(pos);
        motor2.setTargetPosition(pos);
        motor3.setTargetPosition(pos);
    }

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        mark = hardwareMap.get(Servo.class, "mark");

        upServo1 = hardwareMap.get(Servo.class, "upServo1");
        upServo2 = hardwareMap.get(Servo.class, "upServo2");
        roll = hardwareMap.get(CRServo.class, "roll");
        pull = hardwareMap.get(CRServo.class, "pull");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        GoldReconization goldReconization = new GoldReconization(hardwareMap);
        autoWheelBase1 = new AutoWheelBase(LF, LB, RF, RB, gyro, 0.6, 0, 0, 0);
        resetEncoder();

        openloopLift(0);
        upServo1.setPosition(0);
        upServo2.setPosition(1);
        mark.setPosition(0.5);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.addData("motor1", motor1.getCurrentPosition());
        telemetry.addData("motor2", motor2.getCurrentPosition());
        telemetry.update();
        waitForStart();
        state = 1;
        goldPosition = 2;

        if (opModeIsActive()) {
            if (goldReconization.tfod != null) {
                goldReconization.tfod.activate();
            }
            while (opModeIsActive()) {
                autoWheelBase1.compute();
                if (state == 0) {
                    openloopLift(0);
                    if (goldReconization.startReconizing(telemetry).equals("Left")) {
                        goldPosition = 1;
                        state = 1000;
                        goldReconization.endDetect();
                    } else if (goldReconization.startReconizing(telemetry).equals("Center")) {
                        goldPosition = 2;
                        state = 1000;
                        goldReconization.endDetect();
                    } else if (goldReconization.startReconizing(telemetry).equals("Right")) {
                        goldPosition = 3;
                        state = 1000;
                        goldReconization.endDetect();
                    } else if (getRuntime() - timeCounter > 10) {
                        goldPosition = 2;
                        state = 1000;
                    }
                } else if (state == 1000) {

                    upServo1.setPosition(0.5);
                    upServo2.setPosition(0.5);

                    timeCounter = this.getRuntime();

                    state = 1001;
                } else if (state == 1001) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    if (this.getRuntime() - timeCounter > 2) {
                        autoWheelBase1.forward(500);
                        state = 1002;
                    }
                } else if (state == 1002) {
                    autoWheelBase1.forwardUpdate();
                    //motor1.setPower(.7);
                    //motor2.setPower(.7);
                    if (autoWheelBase1.state == 4) {
                        //Reserved
                        state = 1003;
                        //motor1.setPower(0.1);
                        //motor2.setPower(0.1);
                        resetEncoder();
                        autoWheelBase1.compute();
                        autoWheelBase1.sideway(-500);
                        openloopLift(-0.1);
                    }

                } else if (state == 1003) {
                    autoWheelBase1.sidewayUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 1004;
                        openloopLift(0);
                    }
//
                } else if (state == 1004) {
//                    motorBehavior.setPosition(2289);
                    gyro.initialize(parameters);
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(-800);
                    state = 2;
                } else if (state == 2) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 3;
                    }
                } else if (state == 3) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    if (goldPosition == 1) {
                        autoWheelBase1.sideway(-1700);
                        state = 4;
                    } else if (goldPosition == 3) {
                        autoWheelBase1.sideway(2000);
                        state = 4;
                    } else if (goldPosition == 2) {
                        autoWheelBase1.sideway(1);
                        state = 4;
                    }
                } else if (state == 4) {
                    autoWheelBase1.sidewayUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 5;
                    }
//                } else if (state == 4) {
//                    autoWheelBase1.forward(-1000);
//                    if (autoWheelBase1.state == 4) {
//                        state = 5;
//                    }
//                } else if (state == 4) {
//                    autoWheelBase1.forwardUpdate();
//                    if (autoWheelBase1.state == 4) {
//                        state = 5;
//                    }
                } else if (state == 5) {
                    autoWheelBase1.forward(1000);
                    state = 6;
                } else if (state == 6) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 7;
                    }
                } else if (state == 7) {
                    autoWheelBase1.sideway(-800);
                    state = 8;
                } else if (state == 8) {
                    autoWheelBase1.sidewayUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 9;
                    }
                } else if (state == 9) {
                    autoWheelBase1.turn(-150);
                    state = 10;
                }else if (state == 10){
                    autoWheelBase1.turnUpdate();
                    if (autoWheelBase1.state==4){
                        state = 11;
                    }
                }else if (state==11){
                    autoWheelBase1.forward(800);
                    state = 12;
                }else if (state==12){
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state==4){
                        state = 13;
                    }
                } else if (state == 13) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        mark.setPosition(0);
                        state = 14;
                        mark.setPosition(1);
                        timeCounter = getRuntime();
                    }
                } else if (state == 14) {
                    mark.setPosition(0);
                    if (getRuntime() - timeCounter > 1) {
                        state = 15;
                    }
                } else if (state == 15) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(-9000);
                    state = 16;
                } else if (state == 16) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 17;
//                        posCtrlLift(800);
                        timeCounter = getRuntime();
                    }
                }
//                }else if (state == 28){
//                    if (getRuntime() - timeCounter > 1){
//                        pull.setPower(1);
//                        timeCounter = getRuntime();
//                        state = 29;
//                    }
//                }else if (state == 29){
//                    if (getRuntime() - timeCounter > 4){
//                        pull.setPower(0);
//                        openloopLift(0);
//                    }
//                }
                telemetry.addData("base state", autoWheelBase1.state);
                telemetry.addData("state", state);
                telemetry.addData("s", autoWheelBase1.s);
                telemetry.addData("sn", autoWheelBase1.sn);
                telemetry.addData("z: ", autoWheelBase1.getZ());
                telemetry.addData("position: ", goldReconization.startReconizing(telemetry));
                telemetry.addData("Position", goldPosition);
                telemetry.addData("LF", autoWheelBase1.y1);
                telemetry.addData("LB", autoWheelBase1.y2);
                telemetry.addData("RF", autoWheelBase1.y3);
                telemetry.addData("RB", autoWheelBase1.y4);
                telemetry.addData("getY", autoWheelBase1.getY());
                telemetry.update();
            }
        }

    }

    public void resetEncoder() {
        LF.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RF.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

