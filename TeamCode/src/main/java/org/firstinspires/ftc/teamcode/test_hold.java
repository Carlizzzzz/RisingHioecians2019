package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by eitc on 22/5/2019.
 */
@TeleOp(name = "test_hold", group = "Iterative Opmode")
public class test_hold extends setting{

    final double lap =1145;

    @Override
    public void init() {
        telemetry.addData("Status", "Start initialize");

        hold_init();
        holder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();


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
        double quanter = lap/4;

        if (hold) {

                holder.setTargetPosition(550);
                holder.setPower(0.5);



        }
        telemetry.addData("holder Value", holder.getCurrentPosition());
        telemetry.addData("holder power",holder.getPower());
        telemetry.addData("hold",hold);
        telemetry.update();
    }
    @Override
    public void stop() {
    }
}
