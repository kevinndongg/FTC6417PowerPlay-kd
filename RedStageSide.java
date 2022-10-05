package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Stage side")
public class RedStageSide extends LinearOpMode {
    Hardware6417 robot = new Hardware6417();
    int position;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize autonomous and build trajectories
        robot.initAutonomous(hardwareMap, telemetry);

        //build trajectory sequence
        robot.drive.setPoseEstimate(new Pose2d(36, -61, Math.toRadians(90)));

        TrajectorySequence redStageTrajectory = robot.drive.trajectorySequenceBuilder(new Pose2d(36, -61, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, -53), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -36,Math.toRadians(45)), Math.toRadians(90))
                .setVelConstraint(robot.drive.getVelocityConstraint(10, Math.toRadians(120), 10))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(12, -36, Math.toRadians(0)), Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop slider
                })
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24,-36), Math.toRadians(0))
                .setVelConstraint(robot.drive.getVelocityConstraint(10, Math.toRadians(120), 10))
                .forward(6)
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //grab cone after running into it for .5 sec
                })
                .splineToSplineHeading(new Pose2d(36,-36, Math.toRadians(135)), Math.toRadians(0))
                .setVelConstraint(robot.drive.getVelocityConstraint(10, Math.toRadians(120), 10))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(36,-36, Math.toRadians(90)), Math.toRadians(-45))
                .build();

        //telemetry communication
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();

        while (!isStarted()){
            position = robot.pipeline.position;
            telemetry.addData("Position", position);
            telemetry.update();
        }

        //no more need for camera beyond this point
        robot.webcam.stopStreaming();

        if(!isStopRequested()){
            robot.drive.followTrajectorySequence(redStageTrajectory);
        }
    }
}
