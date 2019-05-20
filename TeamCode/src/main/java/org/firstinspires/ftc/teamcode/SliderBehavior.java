package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by teacher on 22-Jan-18.
 */

public class SliderBehavior {
    private DcMotor controlSliderup, controlMotor2;
    private int targetPosition,currentPosition,motorState;
    private double currentVelocity,currentError,lastError, intError;
    private double acc, dam, kp, ki;

    public SliderBehavior(DcMotor Sliderup, double _acc, double _dam, double _kp, double _ki) {
        Sliderup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controlSliderup = Sliderup;
        controlSliderup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            controlSliderup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                controlSliderup.setPower(Range.clip(currentVelocity, -0.7, 0.7));
            }else{
                double power = currentError*kp+(currentError-lastError)*dam+intError*ki;
                controlSliderup.setPower(Range.clip(power, -0.7, 0.7));
            }
            if(Math.abs(targetPosition-takeAverage())<15){
                controlSliderup.setPower(0);
                //motorState=0;
            }
        }
        else if(motorState == 0){

        }else if(motorState==2){
            controlSliderup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            controlSliderup.setPower(0.2);
            controlSliderup.setTargetPosition(targetPosition);
        }
    }

    public int takeAverage(){
        return (controlSliderup.getCurrentPosition()+controlMotor2.getCurrentPosition())/2;
    }

}
