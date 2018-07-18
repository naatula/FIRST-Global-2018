/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.stormbots.MiniPID;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Kulmikas vittu-saatana", group="Linear Opmode")
public class Testi extends LinearOpMode {
    
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
    
    @Override
    public void runOpMode(){
        
        MotorIO mio = new MotorIO(telemetry, hardwareMap);
        Pid pid = new Pid(telemetry, hardwareMap, mio);
        GamepadIO gpio = new GamepadIO(telemetry, hardwareMap, gamepad1, mio, pid);
        Thread gpioT = new Thread(gpio);
        Thread pidT = new Thread(pid);
        Thread mioT = new Thread(mio);
        
        gpioT.start();
        pidT.start();
        mioT.start();
        waitForStart();
        
        while(opModeIsActive()){
            try{
                Thread.sleep(100);
            }catch(InterruptedException e){
                //e.printStackStrace();
            }
        }
        
        gpio.kys();
        pid.kys();
        mio.kys();
    }
}
