package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by teacher on 22-Jan-18.
 */

public class MotorBehavior {
    private DcMotor controlMotor;
    private int targetPosition,currentPosition,motorState;
    private double currentVelocity,currentError,lastError, intError;
    private double acc, dam, kp, ki;

    public MotorBehavior(DcMotor motor, double _acc, double _dam, double _kp, double _ki) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controlMotor = motor;
        controlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acc = _acc;
        dam = _dam;
        kp = _kp;
        ki = _ki;
    }
    public void setState(int state){
        motorState = state;
        // when state is 0, the motor disabled
        // when state is 1, the  motor enabled
    }

    public void setPosition(int position){
        targetPosition=position;
        motorState = 1;
    }
    public void update() {
        if (motorState == 1) {
            controlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentPosition = controlMotor.getCurrentPosition();
            lastError=currentError;
            intError += currentError;
            intError = Range.clip(intError, -100, 100);
            currentError=targetPosition-controlMotor.getCurrentPosition();
            if(targetPosition-currentPosition>0) {
                currentVelocity = Math.sqrt((targetPosition - currentPosition) * acc * 2);
            }else{
                currentVelocity = -Math.sqrt((currentPosition-targetPosition) * acc * 2);
            }
            if(Math.abs(targetPosition-controlMotor.getCurrentPosition())>200){
                controlMotor.setPower(Range.clip(currentVelocity, -0.7, 0.7));
            }else{
                double power = currentError*kp+(currentError-lastError)*dam+intError*ki;
                controlMotor.setPower(Range.clip(power, -0.7, 0.7));
            }
            if(Math.abs(targetPosition-controlMotor.getCurrentPosition())<15){
                controlMotor.setPower(0);
                //motorState=0;
            }
        }
        else if(motorState == 0){

        }else if(motorState==2){
            controlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            controlMotor.setPower(0.2);
            controlMotor.setTargetPosition(targetPosition);
        }
    }

}
