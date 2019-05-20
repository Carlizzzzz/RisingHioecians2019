package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by teacher on 31-Jan-18.
 */

public class AutoWheelBase {
    private DcMotor motor1,motor2,motor3,motor4 = null;
    private BNO055IMU gyro =null;
    private double acc, dam, kp, ki,x,y,z;
    private double fowardTP,sidewayTP,turnTP;
    double angleC = 36.3;
    int state;
    boolean dec;
    double u;
    double sn;
    double vmax = 0.7;
    double v1,v2,v3,v4;
    double vx = 0;
    double vy = 0;
    double s = 0;
    double x1,x2,x3,x4,y1,y2,y3,y4;


    public AutoWheelBase(DcMotor _motor1, DcMotor _motor2, DcMotor _motor3, DcMotor _motor4, BNO055IMU _gyro, double _acc, double _dam, double _kp, double _ki) {
        _motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1 = _motor1;
        motor2 = _motor2;
        motor3 = _motor3;
        motor4 = _motor4;
        _motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acc = _acc;
        dam = _dam;
        kp = _kp;
        ki = _ki;
        gyro = (BNO055IMU) _gyro;
    }

    public void compute(){
        x1 = -motor1.getCurrentPosition();
        x2 = motor2.getCurrentPosition();
        x3 = motor3.getCurrentPosition();
        x4 = -motor4.getCurrentPosition();
        y1 = motor1.getCurrentPosition();
        y2 = motor2.getCurrentPosition();
        y3 = motor3.getCurrentPosition();
        y4 = motor4.getCurrentPosition();

         x = (x1+x2+x3+x4)/4;
         y = (y1+y2+y3+y4)/4;

         z = gyro.getAngularOrientation().firstAngle;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getZ(){
        return z;
    }


    //v1    = Range.clip(x*Math.sin(angleC) - y*Math.cos(angleC)+ turn, -1.0, 1.0) ;
    //v2    = Range.clip(-x*Math.sin(angleC) - y*Math.cos(angleC)+ turn, -1.0, 1.0) ;
    //v3    = Range.clip(-x*Math.sin(angleC) + y*Math.cos(angleC)+ turn, -1.0, 1.0) ;
    //v4    = Range.clip(x*Math.sin(angleC) + y*Math.cos(angleC)+ turn, -1.0, 1.0) ;



    public void sideway(double position){
        fowardTP = position;
        s = (getX() + fowardTP);
        state = 1;
        dec = false;
        u=0;
        vx = 0;
        if (s > getX()){
            vmax = 0.7;
            acc = Math.abs(acc);
        }else{
            vmax = -0.7;
            acc = -Math.abs(acc);
        }
    }

    public void forward(double position){
        sidewayTP = position;
        s = (getY() + sidewayTP);
        state = 1;
        dec = false;
        u=0;
        vy = 0;
        if (s > getY()){
            vmax = 1;
            acc = Math.abs(acc);
        }else{
            vmax = -1;
            acc = -Math.abs(acc);
        }
    }

    public void sidewayUpdate(){
        sn = getX();
        if(state ==1){
            vx = vx + acc;
            if(Math.abs(vx)>Math.abs(vmax)){
                state =2;
                vx = vmax;
            }else{
                v1    = Range.clip(-vx,-0.7,0.7 ) ;
                v2    = Range.clip(vx,-0.7,0.7 ) ;
                v3    = Range.clip(vx,-0.7,0.7 ) ;
                v4    = Range.clip(-vx,-0.7,0.7 ) ;
                motor1.setPower(v1);
                motor2.setPower(v2);
                motor3.setPower(v3);
                motor4.setPower(v4);
            }
        }
        if(state==2){
            u = Math.sqrt(Math.abs(2*acc*(s-sn)));
            if(Math.abs(u)<=Math.abs(vmax)){
                state=3;
            }
            v1    = Range.clip(-vx,-0.7,0.7 ) ;
            v2    = Range.clip(vx,-0.7,0.7 ) ;
            v3    = Range.clip(vx,-0.7,0.7 ) ;
            v4    = Range.clip(-vx,-0.7,0.7 ) ;
            motor1.setPower(v1);
            motor2.setPower(v2);
            motor3.setPower(v3);
            motor4.setPower(v4);
        }
        if(state==3){
            u = Math.sqrt(Math.abs(2*acc*(s-sn)));
            if(s>0){
                vx = u;
            }else{
                vx = u*-1;
            }
            v1    = Range.clip(-vx,-0.7,0.7 ) ;
            v2    = Range.clip(vx,-0.7,0.7 ) ;
            v3    = Range.clip(vx,-0.7,0.7 ) ;
            v4    = Range.clip(-vx,-0.7,0.7 ) ;
            motor1.setPower(v1);
            motor2.setPower(v2);
            motor3.setPower(v3);
            motor4.setPower(v4);
        }
        if(((sn>s) && (s>0)) || ((sn<s) && (s < 0 )) ){
            state = 4;
        }

        if (state==4){
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }

    public void forwardUpdate(){
        sn = getY();
        if(state ==1){
            vy = vy + acc;
            if(Math.abs(vy)>Math.abs(vmax)){
                state =2;
                vy = vmax;
            }else{
                v1    = Range.clip(vy,-0.7,0.7 ) ;
                v2    = Range.clip(vy,-0.7,0.7 ) ;
                v3    = Range.clip(vy,-0.7,0.7 ) ;
                v4    = Range.clip(vy,-0.7,0.7 ) ;
                motor1.setPower(v1);
                motor2.setPower(v2);
                motor3.setPower(v3);
                motor4.setPower(v4);
            }
        }
        if(state==2){
            u = Math.sqrt(Math.abs(2*acc*(s-sn))/100);
            if(Math.abs(u)<=Math.abs(vmax)){
                state=3;
            }
            v1    = Range.clip(vy,-0.7,0.7 ) ;
            v2    = Range.clip(vy,-0.7,0.7 ) ;
            v3    = Range.clip(vy,-0.7,0.7 ) ;
            v4    = Range.clip(vy,-0.7,0.7 ) ;
            motor1.setPower(v1);
            motor2.setPower(v2);
            motor3.setPower(v3);
            motor4.setPower(v4);
        }
        if(state==3){
            u = Math.sqrt(Math.abs(2*acc*(s-sn))/100);
            if(s>0){
                vy = u;
            }else{
                vy = u*-1;
            }
            v1    = Range.clip(vy,-0.7,0.7 ) ;
            v2    = Range.clip(vy,-0.7,0.7 ) ;
            v3    = Range.clip(vy,-0.7,0.7 ) ;
            v4    = Range.clip(vy,-0.7,0.7 ) ;
            motor1.setPower(v1);
            motor2.setPower(v2);
            motor3.setPower(v3);
            motor4.setPower(v4);
        }
        if(((sn>s) && (s>0)) || ((sn<s) && (s < 0 )) ){
            state = 4;
        }

        if (state==4){
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }


    public void turn(double angle){

        while (angle>359){
            angle = angle-360;
        }
        while (angle <0){
            angle = angle + 360;
        }

        turnTP =angle;
        s = turnTP;
        state = 1;
        dec = false;
        u=0;
        vx = 0;
        if (angleCalculation(s,getZ())<0){
            vmax = 0.7;
            acc = Math.abs(acc);
        }else{
            vmax = -0.7;
            acc = -Math.abs(acc);
        }
    }
    public void turnUpdate(){
        sn = getZ();
        if(state ==1){
            vx = vx + acc;
            if(Math.abs(vx)>Math.abs(vmax)){
                state =2;
                vx = vmax;
            }else{
                motor1.setPower(-vx);
                motor2.setPower(-vx);
                motor3.setPower(vx);
                motor4.setPower(vx);
            }
        }
        if(state==2){
            u = Math.sqrt(Math.abs(2*acc*(angleCalculation(s,sn))/50));
            if(Math.abs(u)<=Math.abs(vmax)){
                state=3;
            }
            motor1.setPower(-vx);
            motor2.setPower(-vx);
            motor3.setPower(vx);
            motor4.setPower(vx);
        }
        if(state==3) {
            u = Math.sqrt(Math.abs(2 * acc * (angleCalculation(s,sn))/50));
            if (angleCalculation(s,sn) < 0) {
                vx = u;
            } else {
                vx = u * -1;
            }

            motor1.setPower(-vx);
            motor2.setPower(-vx);
            motor3.setPower(vx);
            motor4.setPower(vx);
        }
        if (Math.abs(angleCalculation(s,sn)) < 1) {
            state = 4;
        }
        if (state==4){
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }

    public double angleCalculation(double a, double b){
        //Range = 0-359
        double c =0;

        while(a < 0){
            a= 360+a;
        }
        while (b<0){
            b = 360+b;
        }
        double  angle1 = a%360;
        double angle2 = b%360;

        if((a-b)>179){
            return (a-b)-360;
        }else if((a-b)<-180){
            return (a-b)+360;
        }else{
            return a-b;
        }

    }
}

