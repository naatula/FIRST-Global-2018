package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.stormbots.MiniPID;

public class Tredi implements Runnable{
    
    Telemetry t;
    bno055driver imu;
    HardwareMap hwmap;
    
    boolean kyss = false;
    
    public void kys(){
        kyss = true;
    }
    
    public Tredi(Telemetry t, bno055driver imu, HardwareMap hwmap){
        this.t = t;
        this.imu = imu;
        this.hwmap = hwmap;
    }

    // todo: write your code here
    
    @Override
    public void run(){
        DcMotor h1 = hwmap.dcMotor.get("h1");
        double c = 1;
        while(!kyss){
            double lastTime = System.nanoTime();
            c = -c;
            h1.setPower(c);
            
            t.addLine("tredi: " + Double.toString(System.nanoTime()-lastTime));
            t.update();
        //    for(int i = 0; i < 1000; i++){
        //        imu.getAngles();
        //    }
        //  t.addLine("Kulmikkuus (vittusaatanan): " + Double.toString((System.nanoTime()-time) / 1000));
        //  t.update();
        //try{
        //  Thread.sleep(10);
        //  
        //}catch(Exception e){
        //}
        }
    }
    
}