package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by eitc on 22/5/2019.
 */
@TeleOp(name = "test_hold", group = "Iterative Opmode")
public class test_hold extends setting {

    final double lap = 1145;
    private MotorBehavior motorBehavior;

    @Override
    public void init() {
        telemetry.addData("Status", "Start initialize");

        hold_init();


        holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBehavior = new MotorBehavior(holder, 0.6 ,0,0, 0);

        runtime.reset();


        telemetry.addData("Status", "Initialized");
        //motorBehavior.setState(2);

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


        if (hold) {
            //holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (gamepad1.dpad_up) {
                motorBehavior.setPosition(550); // Note from Carlix: You may want to use the object "MotorBehavior". MotorBehavior will automatically decelerate the motor when it's near to the target position. So that overshooting can be avoid.


            } else if (gamepad1.dpad_down) {

                motorBehavior.setPosition(100);
                motorBehavior.update();
//                holder.setTargetPosition(10);
//                if (holder.getCurrentPosition() != 10) {
//                    holder.setPower(-0.5);
//                } else {
//                    holder.setPower(0);
//                }
            } else {
                holder.setPower(0);
            }
        }

        telemetry.addData("holder Value", holder.getCurrentPosition());
        telemetry.addData("holder power", holder.getPower());
        telemetry.addData("hold", hold);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
