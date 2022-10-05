package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

            /*CONTROLS
            gamepad 1:
            left stick - left drive
            right stick - right drive
            vertical dpad - slider

            gamepad 2:
            */



@TeleOp(name = "Basic TeleOp", group = "TeleOp")
public class TeleOp6417 extends LinearOpMode{
    Hardware6417 robot      = new Hardware6417();
    ElapsedTime runtime     = new ElapsedTime();

    //power variables
    double drivePower       = ControlConstants.drivePower;
    double sliderPower      = ControlConstants.sliderPower;

    double sens             = ControlConstants.sens;

    double grabberClosePos  = ControlConstants.grabberClosePos;
    double grabberOpenPos   = ControlConstants.grabberOpenPos;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        if(isStopRequested()) return;

        //setup
        robot.start(grabberOpenPos);

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //variables for controlling
        double forwardleft, forwardright, strafeleft, straferight;



        while(opModeIsActive()){
            //control loop

            //input gamepad joystick variables for control
            forwardleft = -1 * Range.clip(gamepad1.left_stick_y,-1,1);
            strafeleft = Range.clip(gamepad1.left_stick_x,-1,1);
            forwardright = -1 * Range.clip(gamepad1.right_stick_y,-1,1);
            straferight = Range.clip(gamepad1.right_stick_x, -1, 1);

            //driving
            {
                //move wheels
                if (Math.abs(forwardleft) > sens || Math.abs(forwardright) > sens || Math.abs(strafeleft) > sens || Math.abs(straferight) > sens) {
                    robot.setDriveSpeeds(drivePower, forwardleft, forwardright, strafeleft, straferight);
                }
                else{
                    robot.stop();
                }
            }

            //slider control
            {
                if(gamepad1.dpad_up){
                    robot.slide(sliderPower);
                }

                if(gamepad1.dpad_down){
                    robot.slide(-sliderPower);
                }
                else if(!gamepad1.dpad_up){
                    robot.slide(0);
                }
            }

            //grabber control
            {
                if(gamepad1.a){
                    robot.closeGrabber();
                }
                if(gamepad1.b){
                    robot.openGrabber();
                }
            }

            //reset stuff
            {
                if(gamepad1.x){
                    robot.resetAngle();
                }
                if(gamepad1.y){
                    robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            //telemetry
            {
                telemetry.addData("left front Pos", robot.leftFront.getCurrentPosition());
                telemetry.addData("right front Pos", robot.rightFront.getCurrentPosition());
                telemetry.addData("left rear Pos", robot.leftRear.getCurrentPosition());
                telemetry.addData("right rear Pos", robot.rightRear.getCurrentPosition());
                telemetry.addData("slider Pos", robot.slider.getCurrentPosition());

                telemetry.addData("Angle", robot.getAngle());

                telemetry.update();
            }
        }
        //stop robot after end
        robot.end();
    }
}