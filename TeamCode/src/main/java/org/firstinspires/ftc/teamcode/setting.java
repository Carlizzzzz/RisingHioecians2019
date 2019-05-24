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


public class setting extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public BNO055IMU gyro;
    public DcMotor
            device,
            LF, LB, RF, RB,
            UpDown, inout, holder, hanger = null;
    public boolean UD, IO, hold, hang, spin, run, flag,put_able;
    public CRServo spinner;
    public Servo put;
    public String name;

    @Override
    public void init() {

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

    public void UD_init() {
        try {
            UpDown = hardwareMap.get(DcMotor.class, "updown");
            UpDown.setDirection(DcMotorSimple.Direction.FORWARD);
            UpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            UD = true;
        } catch (Exception p_exception) {
            UD = false;
            telemetry.addData("Updown", "not founded");
        } finally {
            if (UD) {
                telemetry.addData("Updown", "inited");
            }
        }
//        check(UpDown,UD,"updown");
//        UpDown=device;
//        UD=flag;

    }

    public void IO_init() {
        try {
            inout = hardwareMap.get(DcMotor.class, "inout");
            inout.setDirection(DcMotorSimple.Direction.FORWARD);
            inout.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            IO = true;
        } catch (Exception p_exception) {
            IO = false;
            telemetry.addData("inout", "not founded");
        } finally {
            if (IO) {
                telemetry.addData("inout", "inited");
            }
        }
//        check(inout,IO,"inout");
    }

    public void hold_init() {
        try {
            holder = hardwareMap.get(DcMotor.class, "holder");
            holder.setDirection(DcMotorSimple.Direction.FORWARD);
            holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hold = true;
        } catch (Exception p_exception) {
            hold = false;
            telemetry.addData("holder", "not founded");
        } finally {
            if (hold) {
                telemetry.addData("holder", "inited");
            }
        }
//        check(holder,hold,"holder");
//        holder=device;
//        hold=flag;
    }

    public void hang_init() {
        try {
            hanger = hardwareMap.get(DcMotor.class, "hanger");
            hanger.setDirection(DcMotorSimple.Direction.FORWARD);
            hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hang = true;
        } catch (Exception p_exception) {
            hang = false;
            telemetry.addData("hanger", "not founded");
        } finally {
            if (hang) {
                telemetry.addData("hanger", "inited");
            }
        }
//        check(hanger,hang,"hanger");

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

    public void check(DcMotor _device, boolean _flag, String _name) {//not tested
        try {
            _device = hardwareMap.get(DcMotor.class, _name);
            _device.setDirection(DcMotorSimple.Direction.FORWARD);
            _device.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _flag = true;
        } catch (Exception p_exception) {
            _flag = false;
            telemetry.addData(_device.getDeviceName(), "not founded");
        } finally {
            device = _device;
            flag = _flag;
            name = _name;
            if (flag) {
                telemetry.addData(device.getDeviceName(), "inited");
            }

        }
    }

    public void wheels_init() {
        try {
            LF = hardwareMap.get(DcMotor.class, "LF");
            RB = hardwareMap.get(DcMotor.class, "RB");
            RF = hardwareMap.get(DcMotor.class, "RF");
            LB = hardwareMap.get(DcMotor.class, "LB");
            run = true;
        } catch (Exception p_exception) {
            run = false;
        } finally {
            if (run) {
                LF.setDirection(DcMotor.Direction.FORWARD);
                LB.setDirection(DcMotor.Direction.FORWARD);
                RF.setDirection(DcMotor.Direction.REVERSE);
                RB.setDirection(DcMotor.Direction.REVERSE);

            }
        }
    }

    public void imu_init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        gyro = hardwareMap.get(BNO055IMU.class, "imu");//hardwareMap.get(AdafruitBNO055IMU.class,"imu");
        gyro.initialize(parameters);
    }

    public void put_init(){
        try {
            put = hardwareMap.get(Servo.class,"put");
            put_able= true;
        } catch (Exception p_exception){
            put_able =false;
            telemetry.addData("put","not found");
        } finally {
            if (put_able){
                telemetry.addData("put","inited");
                put.setDirection(Servo.Direction.FORWARD);
            }
        }

    }




}
