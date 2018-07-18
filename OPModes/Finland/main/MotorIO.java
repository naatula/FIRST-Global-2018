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

public class MotorIO implements Runnable{
    
    Telemetry t;
    HardwareMap hwmap;
    
    boolean kyss = false;
    double extension_state = 0;
    double close_state = 0;
    double lift_state = 0;
    double e1 = 0;
    double e2 = 0;
    double e3 = 0;
    
    public void kys(){
        kyss = true;
    }
    
    public MotorIO(Telemetry t, HardwareMap hwmap){
        this.t = t;
        this.hwmap = hwmap;
    }
    
    double clamp(double i, double n, double x){
        return Math.min(Math.max(i,n),x);
    }

    public void update(double extension_state, double close_state, double lift_state){
        this.extension_state = extension_state;
        this.close_state = close_state;
        this.lift_state = lift_state;
    }
    
    public void updateEngineState(double engine1, double engine2, double engine3){
        this.e1 = engine1;
        this.e2 = engine2;
        this.e3 = engine3;
    }

    // todo: write your code here
    
    @Override
    public void run(){
        
        CRServo pidennin = (CRServo) hwmap.get("pidennin");
        DcMotor h1 = hwmap.dcMotor.get("h1");
        DcMotor manta = hwmap.dcMotor.get("m4");
        DcMotor m1 = hwmap.dcMotor.get("m1");
        DcMotor m2 = hwmap.dcMotor.get("m2");
        DcMotor m3 = hwmap.dcMotor.get("m3");
        
        while(!kyss){
            // Koodi
            pidennin.setPower(this.extension_state);
            h1.setPower(-this.close_state);
            manta.setPower(-this.lift_state);
            m1.setPower(clamp(Math.abs(e1), 0.07, 0.8) * Math.signum(e1));
            m2.setPower(clamp(Math.abs(e2), 0.07, 0.8) * Math.signum(e2));
            m3.setPower(clamp(Math.abs(e3), 0.07, 0.8) * Math.signum(e3));
        }
    }
    
}