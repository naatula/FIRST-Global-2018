/*
Copyright 2018 FIRST Tech Challenge Team 250

Made by Zirdi Anoushka and Ursula of Team France First Global 2018

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class OpMode5 extends LinearOpMode {
    private Gyroscope imu;
    private Gyroscope imu_1;
    private DcMotor lift;
    private DcMotor windmill;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private Servo arm;
    private Servo grip;
    private Servo rightPusher;
    private Servo leftPusher;
    private Servo leftIntakeArm;
    private Servo rightIntakeArm;
    private TouchSensor liftTouch;
    boolean intakeArmBool = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double lastUseTime;
    private boolean isUp = false;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        lift = hardwareMap.get(DcMotor.class, "lift");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        arm = hardwareMap.get(Servo.class, "arm");
        grip = hardwareMap.get(Servo.class, "grip");
        rightPusher = hardwareMap.get(Servo.class, "rightPusher");
        leftPusher = hardwareMap.get(Servo.class, "leftPusher");
        leftIntakeArm = hardwareMap.get(Servo.class, "leftIntakeArm");
        rightIntakeArm = hardwareMap.get(Servo.class, "rightIntakeArm");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        arm.setPosition(0);
        grip.setPosition(0);
       // leftIntakeArm.setPosition(0.8);
       // rightIntakeArm.setPosition(0);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        double targetPowerL = 0;
        double targetPowerR = 0;

        while (opModeIsActive()) {
            int position = lift.getCurrentPosition();
        telemetry.addData("Encoder Lift Position", position);

            //arm
            if (this.gamepad1.dpad_up)
                arm.setPosition(1);
            else if (this.gamepad1.dpad_down)
                arm.setPosition(0);

            //grip
            if (this.gamepad1.dpad_left) //up
                grip.setPosition(0);
            else if (this.gamepad1.dpad_right && grip.getPosition() != 0.52) //solar
                grip.setPosition(1);
            else if (this.gamepad1.x) //cube
                grip.setPosition(0.56);

            //intake

            if (this.gamepad1.right_trigger != 0  && !intakeArmBool && runtime.milliseconds() - lastUseTime > 50)
                intakeArmBool = true;
            else if (this.gamepad1.right_trigger != 0  && intakeArmBool && runtime.milliseconds() - lastUseTime > 50)
                intakeArmBool = false;


            if (this.gamepad1.right_trigger != 0){
                lastUseTime = runtime.milliseconds();

            }

            telemetry.addData("runTime", runtime.milliseconds());
                telemetry.addData("lastUseTime", lastUseTime);


            if (intakeArmBool){
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftIntakeArm.setPosition(0.55);
                rightIntakeArm.setPosition(0.45);
                intakeArmBool = true;
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                leftIntakeArm.setPosition(0.43);
                rightIntakeArm.setPosition(0.57);
            }

            telemetry.addData("is Arms On?", intakeArmBool);


            /*outtake
            if (this.gamepad1.left_trigger != 0){
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }*/

            /*intake arm
            if (this.gamepad1.left_trigger != 0){

            }*/

            //left motor
            targetPowerL = -this.gamepad1.left_stick_y * 0.7;
                //slow moving
                if (this.gamepad1.right_bumper && this.gamepad1.left_stick_y < 0)
                {
                targetPowerL = .3;
                }
                else if (this.gamepad1.right_bumper && this.gamepad1.left_stick_y > 0)
                {
                targetPowerL = -.3;
                }
            leftDrive.setPower(targetPowerL);
            telemetry.addData("TargetL Power",targetPowerL);
            telemetry.addData("MotorL Power",leftDrive.getPower());

            //right motor
            targetPowerR = this.gamepad1.left_stick_y * 0.7;
                //slow moving
                if (this.gamepad1.right_bumper  && this.gamepad1.left_stick_y < 0)
                {
                targetPowerR = -.3;
                }
                else if (this.gamepad1.right_bumper  && this.gamepad1.left_stick_y > 0)
                {
                targetPowerR = .3;
                }
            rightDrive.setPower(targetPowerR);
            telemetry.addData("TargetR Power",targetPowerR);
            telemetry.addData("MotorR Power",rightDrive.getPower());


            //turning right
            if (this.gamepad1.right_stick_x > 0){
                leftDrive.setPower(this.gamepad1.right_stick_x);
                rightDrive.setPower(this.gamepad1.right_stick_x);

                telemetry.addData("turning right",-this.gamepad1.right_stick_x);
            }
            //slow turning right
            if (this.gamepad1.right_bumper  && this.gamepad1.right_stick_x > 0) {
                leftDrive.setPower(this.gamepad1.right_stick_x * 0.001);
                rightDrive.setPower(this.gamepad1.right_stick_x * 0.001);

                telemetry.addData("turning right slowly", rightDrive.getPower());
            }

            //turning left
            else if (this.gamepad1.right_stick_x < 0){
                leftDrive.setPower(this.gamepad1.right_stick_x);
                rightDrive.setPower(this.gamepad1.right_stick_x);

                telemetry.addData("turning left", -this.gamepad1.right_stick_x);
            }
            //turning left slowly
            if (this.gamepad1.right_bumper  && this.gamepad1.right_stick_x < 0){
                leftDrive.setPower(this.gamepad1.right_stick_x * 0.001);
                rightDrive.setPower(this.gamepad1.right_stick_x *0.001);

                telemetry.addData("turning left slowly", -leftDrive.getPower());
            }
            else{
             telemetry.addData("turning right",false);
             telemetry.addData("turning left", false);
            }


            /*test motor
            if(this.gamepad1.y == true)
                testMotor.setPower(.2);
            else
                testMotor.setPower(0);*/

             //figuring encoders





            //pushers
            if (this.gamepad1.left_bumper){
                leftPusher.setPosition(.7);
                rightPusher.setPosition(.3);
            }
            else {
                leftPusher.setPosition(1);
                rightPusher.setPosition(0);
            }


            //windmill
            if (this.gamepad1.b == true){
                windmill.setPower(0.2);
                telemetry.addData("Windmill Power", true);
            }
            else {
                windmill.setPower(0);
                telemetry.addData("Windmill Power", false);
            }


            //lift
                //down
                /*
            if(this.gamepad1.a == true && lift.getCurrentPosition() <= 700)
                lift.setPower(.8);
            else
                lift.setPower(0);
                //up
            if (this.gamepad1.y == true && lift.getCurrentPosition()  > -2739)
                lift.setPower(-.8);
            else
                lift.setPower(0);
            */
            telemetry.addData("isUp",isUp);
            if(this.gamepad1.a == true && isUp){
                lift.setTargetPosition(lift.getCurrentPosition() +3400);
                lift.setPower(-0.4);
                isUp = false;
            }

            if (this.gamepad1.y == true && !isUp){
                lift.setTargetPosition(lift.getCurrentPosition() -3400);
                lift.setPower(0.4);
                isUp = true;
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
            }




        }
    }
}
