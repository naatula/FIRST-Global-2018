package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GamepadIO implements Runnable{
    Telemetry telemetry;
    HardwareMap hwmap;
    Gamepad gamepad1;
    MotorIO motorIO;
    Pid pid;
    
    boolean kys_signal = false;
    
    public void kys(){
        this.kys_signal = true;
    }
    public double magnitudeFromVector(double[] vector){
        return Math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]);
    }
    public double angleFromVector(double[] vector){
        double x = vector[0];
        double y = vector[1];
        if(x == 0){
            if(y == 0){
                return 0.0;
            }
            else if(y > 0){
                return Math.PI/2; //90
            }
            else{
                return Math.PI*3/2; //270
            }
        }
        else if(y == 0){
            if(x > 0){
                return 0.0;
            }
            else{
                return Math.PI; //180
            }
        }
        if(x > 0){
            if(y > 0){
                return Math.atan(y/x);
            }
            else{
                return Math.PI*2+Math.atan(y/x);
            }
        }
        else{
            return Math.PI+Math.atan(y/x);
        }
    }
    public double[] vectorFromAngle(double angle, double magnitude){
        return new double[]{Math.cos(angle)*magnitude,Math.sin(angle)*magnitude};
    }
    public GamepadIO(Telemetry telemetry, HardwareMap hwmap, Gamepad gamepad1, MotorIO motorIO, Pid pid){
        this.telemetry = telemetry;
        this.hwmap = hwmap;
        this.gamepad1 = gamepad1;
        this.motorIO = motorIO;
        this.pid = pid;
    }
    
    @Override
    public void run(){
        double x;
        double y;
        double rot;
        double angle;
        double magnitude;
        
        
        double[] raw_vector;
        double[] vector;
        double power = 1;
        int i=0;
        while(!kys_signal){
            try{
                i=i+1;
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                
                if(gamepad1.dpad_right){
                    telemetry.addLine("right");
                    telemetry.update();
                    motorIO.updateMotor(power,0);
                }
                else if(gamepad1.dpad_left){
                    telemetry.addLine("left");
                    telemetry.update();
                    motorIO.updateMotor(-power,0);
                }
                else{
                    telemetry.addLine("no"+Integer.toString(i));
                    telemetry.update();
                    motorIO.updateMotor(0.0,0);
                }
                if(gamepad1.dpad_up){
                    telemetry.addLine("right");
                    telemetry.update();
                    motorIO.updateMotor(power,1);
                }
                else if(gamepad1.dpad_down){
                    telemetry.addLine("left");
                    telemetry.update();
                    motorIO.updateMotor(-power,1);
                }
                else{
                    telemetry.addLine("no");
                    telemetry.update();
                    motorIO.updateMotor(0.0,1);
                }
                if(gamepad1.left_bumper){
                    power = Math.max(0.0,power-0.05);
                }
                if(gamepad1.right_bumper){
                    power = Math.min(1.0,power+0.05);
                }
                raw_vector = new double[]{x,y};
                angle = angleFromVector(raw_vector)+Math.PI/4;
                magnitude = magnitudeFromVector(raw_vector);
                vector = vectorFromAngle(angle, magnitude);
                rot = gamepad1.right_stick_x*0.5;
                
                motorIO.updateMotors(vector[0]+rot,-vector[1]+rot,-vector[0]+rot,vector[1]+rot);
                
                i=i+1;
                
                
                Thread.sleep(50);
            }
            catch(InterruptedException e){
                telemetry.addData("Error", e);

            }
        }
    }
    
}
