package org.firstinspires.ftc.teamcode;

/**
 * Created by eitc on 20/5/2019.
 */

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

import java.util.Map;


public class setting extends OpMode{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor LF, LB, RF, RB,
            UpDown, inout, holder, hanger = null;
    public boolean UD, IO, hold, hang, spin;
    public CRServo spinner;

    @Override
    public void init() {
//        telemetry.addData("Status", "Start initialize");
//
//
//        UD_init();
//        IO_init();
//        hold_init();
//        hang_init();
//
//        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }

    @Override
    public void loop() {
    }
    @Override
    public void stop() {
    }

    public void UD_init(){
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

    public void hang_init() {
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
            telemetry.addData("spinner:", "not founded");
        } finally {
            if (hang) {
                telemetry.addData("spinner:", "inited");
            }
        }
    }

    public void check(Map<DcMotor,CRServo> device,boolean flag,String name){
//        try {
//
//            if ( == device)
//            device = hardwareMap.get(class, name);
//            spinner.setPower(0);
//            spin = true;
//        } catch (Exception p_exception) {
//            hang = false;
//            telemetry.addData("spinner:", "not founded");
//        } finally {
//            if (hang) {
//                telemetry.addData("spinner:", "inited");
//            }
//        }
    }

}
