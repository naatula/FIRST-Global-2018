package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Pid implements Runnable{
    
    Telemetry t;
    HardwareMap hwmap;
    MotorIO motorIO;
    bno055driver d;
    
    //MiniPID pidctrl = new MiniPID(2, 0.0, 7);
    
    boolean kys_signal = false;
    double target = 0;
    double angleComp = Math.toRadians(180);
    double[] vec = {0, 0};
    
    
    public void setTarget(){
        this.target = Math.toRadians(d.getAngles()[0]);
    }
    
    public void kys(){
        kys_signal = true;
    }
    
    public Pid(Telemetry t, HardwareMap hwmap, MotorIO motorIO){
        this.t = t;
        this.hwmap = hwmap;
        this.motorIO = motorIO;
        d = new bno055driver("imu", this.hwmap);
    }
    
    @Override
    public void run(){
        
        
        while(!kys_signal){
            double trueAngle = Math.toRadians(d.getAngles()[0]);
            t.addData("pos",trueAngle);
            t.update();
            if(Math.abs(trueAngle-target)>0.1){
                motorIO.setRotation(trueAngle-target);
            }
            else{
                motorIO.setRotation(0.0);
            }
         }
         
    }
    
}