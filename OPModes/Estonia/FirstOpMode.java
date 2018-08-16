package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT;
import static com.sun.tools.doclint.Entity.and;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

/**
 * Created by tjones on 4/3/2017.
 */



@TeleOp(name = "vant", group = "Practice-Bot")
public class FirstOpMode extends LinearOpMode {
    private DcMotor NW_Motor;
    private DcMotor NE_Motor;
    private DcMotor SE_Motor;
    private DcMotor SW_Motor;
    private DcMotor Kapp;
    private CRServo tuulik;
    private DcMotor lift1;
    private DcMotor lift2;
    private DigitalChannel hall;


    private ElapsedTime period = new ElapsedTime();

    /*** *
     * waitForTick implements a periodic delay. However, this acts like a metronome
     * with a regular periodic tick. This is used to compensate for varying
     * processing times for each cycle. The function looks at the elapsed cycle time,
     * and sleeps for the remaining time interval.
     * * */

    private void waitForTick(long periodMs) throws java.lang.InterruptedException {
        long remaining = periodMs - (long) period.milliseconds();
// sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            Thread.sleep(remaining);
        }


// Reset the cycle clock for the next pass.
        period.reset();
    }



    public void runOpMode() {

        //teeme pidi



        float moodul;
        //long aeg= System.nanoTime();
        float lift_speed = 0.0f;
        float kapp_pos = 0.0f;
        boolean kiiresti=true;
        boolean haare=false;

        hall = hardwareMap.digitalChannel.get("hall");

        NW_Motor = hardwareMap.dcMotor.get("nw"); //Port 0
        NE_Motor = hardwareMap.dcMotor.get("ne"); //Port 1
        SE_Motor = hardwareMap.dcMotor.get("se"); //Port 2
        SW_Motor = hardwareMap.dcMotor.get("sw"); //Port 3

        Kapp = hardwareMap.dcMotor.get("aivarvinne");
        tuulik = hardwareMap.crservo.get("servo1");
        lift1 = hardwareMap.dcMotor.get("lift1"); //port1
        lift2 = hardwareMap.dcMotor.get("lift2"); //port2

        NW_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        SW_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);



        NW_Motor.setZeroPowerBehavior(BRAKE);
        SW_Motor.setZeroPowerBehavior(BRAKE);
        NE_Motor.setZeroPowerBehavior(BRAKE);
        SE_Motor.setZeroPowerBehavior(BRAKE);

        lift1.setZeroPowerBehavior(BRAKE);
        lift2.setZeroPowerBehavior(BRAKE);
        //Kapp.setZeroPowerBehavior(BRAKE);

        // Set all motors to zero power
        NW_Motor.setPower(0);
        NE_Motor.setPower(0);
        SE_Motor.setPower(0);
        SW_Motor.setPower(0);
        lift1.setPower(-0.3);
        lift2.setPower(-0.3);
        hall.setMode(INPUT);

        while(hall.getState()){telemetry.addData("magnet",hall.getState());telemetry.update();}
        lift1.setPower(0);
        lift2.setPower(0);

        lift1.setMode(STOP_AND_RESET_ENCODER);
        lift2.setMode(STOP_AND_RESET_ENCODER);
        lift1.setMode(RUN_TO_POSITION);
        lift2.setMode(RUN_TO_POSITION);

        lift1.setPower(lift_speed);
        lift2.setPower(lift_speed);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        Kapp.setPower(kapp_pos);
        tuulik.setPower(0);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "kaka");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        try {
            //run until
            while (opModeIsActive()) {
                /*
                // Run wheels in tank mode (note: The joystick goes negative when pushed forwards,
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
                leftMotor.setPower(left);
                rightMotor.setPower(right);
                */
                float LX =-gamepad1.left_stick_x;
                float LY =-gamepad1.left_stick_y;
                float RX =-gamepad1.right_stick_x;
                boolean haara=gamepad1.right_bumper;
                boolean lahti=gamepad1.left_bumper;
                boolean kiire=gamepad1.y;
                boolean aeglane=gamepad1.x;
                boolean lifti_ules = gamepad1.dpad_up;
                boolean lifti_alla = gamepad1.dpad_down;
                boolean vastu= gamepad1.dpad_left;
                boolean pari= gamepad1.dpad_right;


                /*
                NE_Motor.setPower(LX);
                SW_Motor.setPower(LX);
                SE_Motor.setPower(LY);
                NW_Motor.setPower(LY);
                */

                if(abs(LX)<abs(LY)) {
                    moodul = (float)Math.sin((float) Math.sqrt(LY*LY+LX*LX))/((float) Math.sqrt(1+LX*LX));
                } else {
                    moodul = (float)Math.sin((float) Math.sqrt(LY*LY+LX*LX))/((float) Math.sqrt(1+LY*LY));
                }

                if(kiire){
                    kiiresti=true;

                }
                else if(aeglane){
                    kiiresti=false;
                }telemetry.addData("kiirestsi",kiiresti);

                telemetry.addData("moodul",moodul);
                if(!kiiresti){
                    moodul=0.4f;

                } else{
                    moodul=1;
                }

                telemetry.addData("munn",moodul);
                telemetry.addData("LX",LX);
                telemetry.addData("LY",LY);
                telemetry.addData("RX",RX);
                telemetry.addData("dpad_up",lifti_ules);
                telemetry.addData("dpad_down",lifti_alla);
                telemetry.addData("lifti väärtus1",lift1.getCurrentPosition());
                telemetry.addData("lifti väärtus2",lift2.getCurrentPosition());


                //Paremal on x ja y vahetuses

                double SW_Power = 0.0;
                double SE_Power = 0.0;
                double NW_Power = 0.0;
                double NE_Power = 0.0;

                //Siit algab lineaarse liikumise kood
                if(LX==0 && LY==0){
                    SW_Power=0;
                    NE_Power=0;
                    NW_Power=0;
                    NE_Power=0;

                } else if(LX==0) {
                    SE_Power = Integer.signum((int) Math.round(LY*1000000)) * moodul;
                    NW_Power = Integer.signum((int) Math.round(LY*1000000)) * moodul;

                } else if(LY==0){
                    SW_Power = Integer.signum((int) Math.round(LX*1000000)) * moodul;
                    NE_Power = Integer.signum((int) Math.round(LX*1000000)) * moodul;
                } else {
                    SW_Power = Integer.signum((int) Math.round(LX*1000000)) * moodul / ((float) Math.sqrt(1 + (LY / LX)*(LY / LX)));
                    NE_Power = Integer.signum((int) Math.round(LX*1000000)) * moodul / ((float) Math.sqrt(1 + (LY / LX)*(LY / LX)));

                    SE_Power = Integer.signum((int) Math.round(LY*1000000)) * moodul / ((float) Math.sqrt(1 + (LX / LY)*(LX / LY)));
                    NW_Power = Integer.signum((int) Math.round(LY*1000000)) * moodul / ((float) Math.sqrt(1 + (LX / LY)*(LX / LY)));
                }

                //siin lõppeb

                //siin algab pööramise osa



                double poordKiirus = 0;
                if(moodul==0){
                    poordKiirus=RX;

                } else{
                    poordKiirus=moodul*RX;
                }
                SW_Power-=0.5*poordKiirus;
                NE_Power+=0.5*poordKiirus;
                NW_Power-=0.5*poordKiirus;
                SE_Power+=0.5*poordKiirus;
                /*
                if(moodul>0.7 || moodul<-0.7){
                    SW_Power=0.5/0.3*(SW_Power-0.7)+0.5;
                    NE_Power=0.5/0.3*(NE_Power-0.7)+0.5;
                    NW_Power=0.5/0.3*(NW_Power-0.7)+0.5;
                    SE_Power=0.5/0.3*(SE_Power-0.7)+0.5;
                }
                else{
                    SW_Power*=0.5/0.7;
                    NE_Power*=0.5/0.7;
                    NW_Power*=0.5/0.7;
                    SE_Power*=0.5/0.7;
                }*/

                //Siin lõppeb pööramise osa

                //Siin on mootorite värgendused
                SW_Motor.setPower(SW_Power);
                SE_Motor.setPower(SE_Power);
                NW_Motor.setPower(NW_Power);
                NE_Motor.setPower(NE_Power);
                telemetry.addData("mootor",SW_Power);
                //Siit algab servo kood

                /*if(haara){
                    Kapp.setPower(1);
                }
                if(lahti){
                    Kapp.setPower(-1);
                }*/
                if(haara){
                    kapp_pos = 0.2f;
                    haare=true;
                }
                else if(lahti) {
                    kapp_pos = -0.2f;
                    haare=false;
                }


                else if(haare){
                    kapp_pos = 0.5f;
                }
                else{
                    kapp_pos=0.0f;

                }

                Kapp.setPower(kapp_pos);

                //siit algab lifti koodg
                /*if(System.nanoTime()-aeg>1200000000){
                    lift1.setPower(0);
                    lift2.setPower(0);
                    telemetry.addData("Timeout",0);
                }
                if(lifti_ules){
                    lift1.setTargetPosition(1900);
                    lift2.setTargetPosition(1900);
                    lift1.setPower(0.6);
                    lift2.setPower(0.6);
                    aeg= System.nanoTime();

                }
                if(lifti_alla){
                    lift1.setTargetPosition(150);
                    lift2.setTargetPosition(150);
                    lift1.setPower(0.4);
                    lift2.setPower(0.4);
                    aeg= System.nanoTime();

                }*/
                if(lifti_ules){
                    lift1.setTargetPosition(2310);
                    lift2.setTargetPosition(2310);
                    lift_speed += 0.02f;

                } else if(lifti_alla){
                    lift1.setTargetPosition(0);
                    lift2.setTargetPosition(0);
                    lift_speed += 0.02f;
                } else{
                    lift_speed = 0.0f;
                }
                lift1.setPower(lift_speed);
                lift2.setPower(lift_speed);

                if(pari){
                    tuulik.setPower(-1);
                }
                else if(vastu){
                    tuulik.setPower(1);
                }
                else{
                    tuulik.setPower(0);
                }
                //SEDA SITTA EI NÄPI, SEE ON TÄHTIS!!!!!!
                telemetry.update();

                // Pause for metronome tick. 40 mS each cycle = update 25 times a second.

                waitForTick(1);

            }
        } catch (java.lang.InterruptedException exc) {
            return;
        } finally {
            NW_Motor.setPower(0);
            NE_Motor.setPower(0);
            SE_Motor.setPower(0);
            SW_Motor.setPower(0);
            Kapp.setPower(0);
        }
    }
}

class PID{
    double P;
    double I;
    double D;

    double current=0.0;
    double integral=0;

    public PID(double P_in,double I_in,double D_in){
        P=P_in;
        I=I_in;
        D=D_in;
    }
    public ArrayList out(){
        ArrayList _out = new ArrayList();
        _out.add(P);
        _out.add(I);
        _out.add(D);
        return _out;
    }

    public double do_da_pid(double target){
        double difference=target-current;
        integral+=difference;
        current+=P*difference+integral*I;
        return current;
    }

}
