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


public class Pid implements Runnable{
    
    Telemetry t;
    HardwareMap hwmap;
    MotorIO mio;
    bno055driver d;
    
    MiniPID pidctrl = new MiniPID(2, 0.0, 7);
    
    boolean kyss = false;
    double tgtAngle = 0;
    double angleComp = Math.toRadians(180);
    double[] vec = {0, 0};
    
    double tgtRot = 0;
    
    public void kys(){
        kyss = true;
    }
    
    public Pid(Telemetry t, HardwareMap hwmap, MotorIO mio){
        this.t = t;
        this.hwmap = hwmap;
        this.mio = mio;
        d = new bno055driver("imu", this.hwmap);
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
    
    public synchronized void angleDelta(double delta){
        tgtAngle += delta;
    }
    
    public void setVector(double x, double y){
        this.vec[0] = x;
        this.vec[1] = y;
    }
    
    public void setRot(double d){
        this.tgtRot = d;
    }
    
    public void resetAngle(){
        this.angleComp = Math.toRadians(d.getAngles()[0] + 180);
    }
    
    
    
    @Override
    public void run(){
        double prev = System.nanoTime();
        double max_so_far = 0;
        double max_t = 0;
        while(!kyss){
            double trueAngle = Math.toRadians(d.getAngles()[0]);
            double[] localInput = toLocalCoordinates(vec, trueAngle - this.angleComp);
            final double[][] engine_vectors = new double[][]{
              new double[]{1, 0},
              new double[]{-0.5,  0.8660254038},
              new double[]{-0.5, -0.8660254038}
            };
            t.addLine(Double.toString(localInput[0]) + " " + Double.toString(localInput[1]));
            //t.update();
            double angleDelta = -this.tgtRot;//-1 * (pidctrl.getOutput(trueAngle, tgtAngle) / 360);
            double[] motor_proportions = {angleDelta, angleDelta, angleDelta};
            for(int i = 0; i < 3; i++){

              // Dot the input vector with the engine directions to do a component decomposition
              motor_proportions[i] = engine_vectors[i][0] * localInput[0] + engine_vectors[i][1] * (-localInput[1]);
              motor_proportions[i] = lengthOf(vec) * motor_proportions[i]*0.45 + angleDelta;
                //clamp(output,-1.0,1.0)*0.2); // pid control output
                
                //telemetry.addLine("Motor #"+Integer.toString(i+1)+": "+Double.toString(motor_proportions[i]));
            }
            mio.updateEngineState(motor_proportions[0], motor_proportions[1], motor_proportions[2]);
            double cur = System.nanoTime();
            if(cur - max_t > 1 || max_so_far < cur - prev){
                max_t = cur;
                max_so_far = cur - prev;
            }
            t.addLine(Double.toString(max_so_far));
            t.addLine(Double.toString(pidctrl.P) + " " + Double.toString(pidctrl.I) + " " + Double.toString(pidctrl.D));
            t.update();
            prev = cur;
            //t.addLine(Double.toString(pidctrl.getOutput(trueAngle, tgtAngle)));
            //t.addLine(Double.toString(trueAngle));
            //t.update();
        }
    }
    
    /*
    
            double prev = System.nanoTime();
        double max_so_far = 0;
        double max_t = 0;
        while(!kyss){
            double extension_state = gamepad1.right_stick_x;
            double close_state = gamepad1.right_trigger - 0.25*gamepad1.left_trigger;
            double lift_state = gamepad1.right_stick_y;
            
            
            mio.update(extension_state, close_state, lift_state);
            
            
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
    
    */
    
}