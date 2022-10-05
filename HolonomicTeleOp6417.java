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
            a - close grabber
            b - open grabber
            y - reset angle
            x - reset slider encoder
            dpad left - toggle maintain heading

            gamepad 2:
            */



@TeleOp(name = "Holonomic TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode{
    Hardware6417 robot = new Hardware6417();
    ElapsedTime runtime = new ElapsedTime();

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
        double verticalDrive, horizontalDrive, rotateDrive;
        boolean maintainHeading = true;
        boolean lastDpadLeft = false;

        while(opModeIsActive()){
            //control loop

            //input gamepad joystick variables for control
            verticalDrive = -1 * Range.clip(gamepad1.left_stick_y,-1,1);
            horizontalDrive = Range.clip(gamepad1.left_stick_x,-1,1);
            rotateDrive = Range.clip(gamepad1.right_stick_x, -1, 1);

            //driving
            {
                //move wheels
                if (Math.abs(verticalDrive) > sens || Math.abs(horizontalDrive) > sens || Math.abs(rotateDrive) > sens) {
                    robot.holonomicDrive(drivePower,horizontalDrive,verticalDrive,rotateDrive,maintainHeading);
                }
                else{
                    robot.stop();
                }

                //toggle maintain heading
                if(gamepad1.dpad_left && !lastDpadLeft){
                    maintainHeading = !maintainHeading;
                }

                lastDpadLeft = gamepad1.dpad_left;
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
                if(gamepad1.y){
                    robot.resetAngle();
                }
                if(gamepad1.x){
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

                telemetry.addData("Angle", Math.toDegrees(robot.getAngle()));
                telemetry.addData("maintain heading", maintainHeading);

                telemetry.update();
            }
        }
        //stop robot after end
        robot.end();
    }
}
