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

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
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
    private DcMotor LF, LB, RF, RB,
            UpDown, inout, holder,hanger = null;
    private boolean UD,IO,hold,hang,spin;
    public CRServo spinner;
    double timeCounter = 0;


    public void UD_init() {
        try {
            UpDown = hardwareMap.get(DcMotor.class, "updown");
            UpDown.setDirection(DcMotorSimple.Direction.FORWARD);
            UpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            UD = true;
        } catch (Exception p_exception) {
            UD = false;
            telemetry.addData("Updown:", "not founded");
        } finally {
            if (UD) {
                telemetry.addData("Updown:", "inited");
            }
        }
    }

    public void IO_init() {
        try {
            inout = hardwareMap.get(DcMotor.class, "inout");
            inout.setDirection(DcMotorSimple.Direction.FORWARD);
            inout.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            IO = true;
        } catch (Exception p_exception) {
            IO = false;
            telemetry.addData("inout:", "not founded");
        } finally {
            if (IO) {
                telemetry.addData("inout:", "inited");
            }
        }
    }

    public void hold_init() {
        try {
            holder = hardwareMap.get(DcMotor.class, "holder");
            holder.setDirection(DcMotorSimple.Direction.FORWARD);
            holder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hold = true;
        } catch (Exception p_exception) {
            hold = false;
            telemetry.addData("holder:", "not founded");
        } finally {
            if (hold) {
                telemetry.addData("holder:", "inited");
            }
        }
    }

    public void hang_init(){
        try {
            hanger = hardwareMap.get(DcMotor.class, "hanger");
            hanger.setDirection(DcMotorSimple.Direction.FORWARD);
            hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hang = true;
        } catch (Exception p_exception) {
            hang = false;
            telemetry.addData("hanger:", "not founded");
        } finally {
            if (hang) {
                telemetry.addData("hanger:", "inited");
            }
        }
    }

    public void spin_init() {
        try {
            spinner = hardwareMap.get(CRServo.class, "spinner");
            spinner.setPower(0);
            spin = true;
        } catch (Exception p_exception) {
            hang = false;
            telemetry.addData("spinner", "not founded");
        } finally {
            if (hang) {
                telemetry.addData("spinner", "inited");
            }
        }
    }

    @Override
    public void runOpMode() {

        //checker = new check (UpDown, inout, holder,hanger);

        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        gyro = hardwareMap.get(BNO055IMU.class, "imu");//hardwareMap.get(AdafruitBNO055IMU.class,"imu");
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        UD_init();
        IO_init();
        hold_init();
        hang_init();
        hanger.setPower(-1);




        GoldReconization goldReconization = new GoldReconization(hardwareMap);
        autoWheelBase1 = new AutoWheelBase(LF, LB, RF, RB, gyro, 0.6, 0, 0, 0);
        resetEncoder();



        telemetry.addData(">", "Press Play to start tracking");

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
                if (state == 1) {                           // Don't forget to put down the robot b4 doing anything!
                    hanger.setPower(1);     //See where the camera will be facing. If camera is already facing the minerals, then it's unnecessary
                    state = 2;                         // Try your best to avoid using sideway. The error is rather big
                } else if(state == 2){
                    if (goldReconization.startReconizing(telemetry).equals("Left")) {
                        goldPosition = 1;
                        goldReconization.endDetect();
                        state = 3;
                    } else if (goldReconization.startReconizing(telemetry).equals("Center")) {
                        goldPosition = 2;
                        goldReconization.endDetect();
                        state = 3;
                    } else if (goldReconization.startReconizing(telemetry).equals("Right")) {
                        goldPosition = 3;
                        goldReconization.endDetect();
                        state = 3;
                    }
                } else if (state == 3) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.turn(90);
                    state = 4;
                } else if (state == 4) {
                    timeCounter = this.getRuntime();
                    autoWheelBase1.turnUpdate();
                    state = 5;
                } else if (state == 5){
                    hanger.setPower(0);
                    state = 6;
                } else if (state == 6) {
                    resetEncoder();
                    autoWheelBase1.compute();

                    if (this.getRuntime() - timeCounter > 2) {
                        holder.setPower(-0.4);
                        state = 7;
                    }
                } else if (state == 7){
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.turn(1);
                    state = 8;
                } else if (state == 8) {
                    autoWheelBase1.turnUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 9;
                        holder.setPower(0);
                        resetEncoder();
                        autoWheelBase1.compute();
                        autoWheelBase1.turn(90);

                    }
                }else if (state == 9){
                    autoWheelBase1.turnUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 10;

                    }
                }else if (state == 10){
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(-700);
                    state = 11;
                }else if (state == 11){
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 12;

                    }
                } else if (state == 12) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    if (goldPosition == 1) {
                        autoWheelBase1.sideway(-1300);
                        state = 13;
                    } else if (goldPosition == 3) {
                        autoWheelBase1.sideway(2000);
                        state = 13;
                    } else if (goldPosition == 2) {
                        autoWheelBase1.sideway(200);
                        state = 13;
                    }
                } else if (state == 13) {
                    autoWheelBase1.sidewayUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 14;
                    }

                } else if (state == 14) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(-900);
                    if (autoWheelBase1.state == 4) {
                        state = 15;
                    }
                } else if (state == 15) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 16;
                    }
                } else if (state == 16) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(900);
                    state = 17;
                } else if (state == 17) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 18;
                    }
                } else if(state == 18) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.turn(90);
                    state = 19;
                } else if (state == 19){
                    autoWheelBase1.turnUpdate();
                    state = 20;
                } else if (state == 20){
                    resetEncoder();
                    autoWheelBase1.compute();
                    if (goldPosition == 1) {
                        autoWheelBase1.forward(-4000);
                        state = 21;
                    } else if (goldPosition == 3) {
                        autoWheelBase1.forward(-3500);
                        state = 21;
                    } else if (goldPosition == 2) {
                        autoWheelBase1.forward(-3000);
                        state = 21;
                    }
                } else if (state == 21){
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state==4){
                        state = 22;
                    }
                } else if (state == 22) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(300);
                    state = 23;
                } else if (state == 23) {
                    autoWheelBase1.forwardUpdate();
                    state = 24;
                } else if (state == 24) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.turn(-135);
                    state = 25;
                } else if (state == 25){
                    autoWheelBase1.turnUpdate();
                    state = 26;
                } else if (state == 26){
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(-2000);
                    state = 27;
                } else if (state == 27) {
                    autoWheelBase1.forwardUpdate();
                    state = 28;
                }else if (state == 28){
                    spinner.setPower(1);
                    if (autoWheelBase1.state==4){
                        state = 29;}
                } else if (state == 29) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(0);
                    spinner.setPower(0);
                    state = 30;
                } else if (state == 30) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 31;
                    }
                } else if (state == 31) {
                    resetEncoder();
                    autoWheelBase1.compute();
                    autoWheelBase1.forward(9000);
                    state = 32;
                } else if (state == 32) {
                    autoWheelBase1.forwardUpdate();
                    if (autoWheelBase1.state == 4) {
                        state = 33;
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
}


    public void resetEncoder() {
        LF.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RF.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RB.setMode(DcMotor.RunMode.RESET_ENCODERS);


        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}

