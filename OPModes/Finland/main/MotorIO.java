package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorIO implements Runnable{
    
    Telemetry telemetry;
    HardwareMap hwmap;
    
    boolean kys_signal = false;
    double motor_a_power = 0;
    double motor_b_power = 0;
    double motor_c_power = 0;
    double motor_d_power = 0;
    
    double motor_0_power = 0;
    double motor_1_power = 0;
    double motor_2_power = 0;
    double motor_3_power = 0;
    
    double rotation = 0;

    public void kys(){
        kys_signal = true;
    }
    
    public MotorIO(Telemetry telemetry, HardwareMap hwmap){
        this.telemetry = telemetry;
        this.hwmap = hwmap;
    }
    
    public void updateMotors(double motor_a_power, double motor_b_power, double motor_c_power, double motor_d_power){
        this.motor_a_power = motor_a_power;
        this.motor_b_power = motor_b_power;
        this.motor_c_power = motor_c_power;
        this.motor_d_power = motor_d_power;
    }
    public void setRotation(double rotation){
        this.rotation = rotation;
        
    }
    public void updateMotor(double motor_power, int index){
        if(index == 0){
            this.motor_0_power = motor_power;
        }
        else if(index == 1){
            this.motor_1_power = motor_power;
        }
        else if(index == 2){
            this.motor_2_power = motor_power;
        }
        else if(index == 3){
            this.motor_3_power = motor_power;
        }
    }
    @Override
    public void run(){
        DcMotor motor_a = hwmap.dcMotor.get("A");
        DcMotor motor_b = hwmap.dcMotor.get("B");
        DcMotor motor_c = hwmap.dcMotor.get("C");
        DcMotor motor_d = hwmap.dcMotor.get("D");
        
        DcMotor motor_0 = hwmap.dcMotor.get("0");
        DcMotor motor_1 = hwmap.dcMotor.get("1");
        DcMotor motor_2 = hwmap.dcMotor.get("2");
        DcMotor motor_3 = hwmap.dcMotor.get("3");

        while(!kys_signal){/*
            motor_a.setPower(motor_a_power + rotation);
            motor_b.setPower(motor_b_power + rotation);
            motor_c.setPower(motor_c_power + rotation);
            motor_d.setPower(motor_d_power + rotation);
            
            motor_0.setPower(motor_0_power);
            motor_1.setPower(motor_1_power);
            motor_2.setPower(motor_2_power);
            motor_3.setPower(motor_3_power);
            */
            motor_a.setPower(rotation);
        }
    }
}
