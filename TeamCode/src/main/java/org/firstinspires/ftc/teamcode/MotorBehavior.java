package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by teacher on 22-Jan-18.
 */

public class MotorBehavior {
    private DcMotor controlMotor1, controlMotor2;
    private int targetPosition,currentPosition,motorState;
    private double currentVelocity,currentError,lastError, intError;
    private double acc, dam, kp, ki;

    public MotorBehavior(DcMotor motor1, DcMotor motor2, double _acc, double _dam, double _kp, double _ki) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controlMotor1 = motor1;
        //controlMotor2 = motor2;
        controlMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //controlMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            controlMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //controlMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentPosition = takeAverage();
            lastError=currentError;
            intError += currentError;
            intError = Range.clip(intError, -100, 100);
            currentError=takeAverage();
            if(targetPosition-currentPosition>0) {
                currentVelocity = Math.sqrt((targetPosition - currentPosition) * acc * 2);
            }else{
                currentVelocity = -Math.sqrt((currentPosition-targetPosition) * acc * 2);
            }
            if(Math.abs(targetPosition-takeAverage())>200){
                controlMotor1.setPower(Range.clip(currentVelocity, -0.7, 0.7));
                //controlMotor2.setPower(Range.clip(currentVelocity, -0.7, 0.7));
            }else{
                double power = currentError*kp+(currentError-lastError)*dam+intError*ki;
                controlMotor1.setPower(Range.clip(power, -0.7, 0.7));
                //controlMotor2.setPower(Range.clip(power, -0.7, 0.7));
            }
            if(Math.abs(targetPosition-takeAverage())<15){
                controlMotor1.setPower(0);
                //controlMotor2.setPower(0);
                //motorState=0;
            }
        }
        else if(motorState == 0){

        }else if(motorState==2){
            controlMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //controlMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            controlMotor1.setPower(0.2);
            //controlMotor2.setPower(0.2);
            controlMotor1.setTargetPosition(targetPosition);
            //controlMotor2.setTargetPosition(targetPosition);
        }
    }

    public int takeAverage(){
        return (controlMotor1.getCurrentPosition()/*+controlMotor2.getCurrentPosition()*/)/2;
    }

}
