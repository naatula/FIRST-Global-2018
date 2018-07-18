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

public class GamepadIO implements Runnable{
    
    Telemetry t;
    HardwareMap hwmap;
    Gamepad gamepad1;
    MotorIO mio;
    Pid pid;
    
    boolean kyss = false;
    
    public void kys(){
        kyss = true;
    }
    public static double[] vecFor(double angle){
        return new double[]{Math.sin(angle), Math.cos(angle)};
    }

    public static double[] normalize(double[] input){
        double c = Math.sqrt(input[0]*input[0] + input[1]*input[1]);
        if(c != 0){
            return new double[]{input[0]/c, input[1]/c};
        }else{
            throw new ArithmeticException("Division by zero");
        }
    }

    public static double angleFrom(double[] input) throws ArithmeticException{
        double[] vec = normalize(input);
        double base = Math.acos(vec[1]);
        if(vec[0] < 0){
            base = Math.PI*2 - Math.acos(vec[1]);
        }
        if(vec[1] == 0){
            if(vec[0] > 0){
                return Math.PI / 2;
            }else if(vec[0] < 0){
                return -Math.PI / 2;
            }else{
                throw new ArithmeticException("Angle for zero vector.");
            }
        }
        return base;
    }

    double lengthOf(double[] vector){
        return Math.sqrt(vector[0]*vector[0] + vector[1]*vector[1]);
    }

    double[] toLocalCoordinates(double[] inputVector, double globalRotation){
        if(lengthOf(inputVector) != 0) { // The angle is only defined for a non-null vector
            return vecFor(angleFrom(inputVector) - globalRotation);
        }else{
            return new double[]{0, 0};
        }
    }
    
    double map(double n, double a, double b, double c, double d){
        return c+(n-a)/(b-a)*(d-c);
    }
    double expo(double i, double e, double mapmin, double mapmax){
        return Math.signum(i)*map(Math.pow(Math.abs(i),e),0.0,1.0,mapmin,mapmax);
        //return Math.signum(i)*Math.pow(Math.abs(i), e);
    }
    double expomid(double i, double e, double c){
        double gx = 1/(1-c)*(i-c);
        double gminusx = 1/(1-c)*(-i-c);
        double fx = Math.signum(gx)*Math.pow(Math.abs(gx), e);
        double fminusx = Math.signum(gminusx)*Math.pow(Math.abs(gminusx), e);
        double f1 = Math.signum(1)*Math.pow(Math.abs(1), e);
        double f0 = Math.signum(-c/(1-c))*Math.pow(Math.abs(-c/(1-c)), e);
        double s = f0 - f1;
        double hx = (fx - f0)/s;
        double hminusx = (fminusx - f0)/s;
        double px = (Math.signum(hx)+1)/2;
        double pminusx=1-px;
        return px*hx - pminusx*hminusx;
    }
    double clamp(double i, double n, double x){
        return Math.min(Math.max(i,n),x);
    }
    
    public GamepadIO(Telemetry t, HardwareMap hwmap, Gamepad gamepad1, MotorIO mio, Pid pid){
        this.t = t;
        this.hwmap = hwmap;
        this.gamepad1 = gamepad1;
        this.mio = mio;
        this.pid = pid;
    }

    // todo: write your code here
    
    @Override
    public void run(){
        double prev = System.nanoTime();
        double max_so_far = 0;
        double max_t = 0;
        while(!kyss){
            double extension_state = gamepad1.right_stick_x;
            double close_state = gamepad1.right_trigger - 0.25*gamepad1.left_trigger;
            double lift_state = 0.5 * (Math.pow((gamepad1.right_stick_y * -1 + 1), 1.5) - 1);
            
            
            
            
            
            /*if(gamepad1.dpad_up) pid.addP();
            else if(gamepad1.dpad_down) pid.subP();*/
            pid.setRot(0.3 * ((gamepad1.left_bumper ? 1 : 0) - (gamepad1.right_bumper ? 1 : 0) ));
            pid.setVector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            mio.update(extension_state, close_state, lift_state);
            if(gamepad1.a) pid.resetAngle();
            
            double cur = System.nanoTime();
            if(cur - max_t > 1000000000 || max_so_far < cur - prev){
                max_t = cur;
                max_so_far = cur - prev;
            }
            //t.addLine(Double.toString(max_so_far));
            //t.update();
            prev = cur;
        }
    }
    
}