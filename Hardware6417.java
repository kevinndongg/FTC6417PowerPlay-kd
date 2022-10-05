package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are uppercase
 *
 * Motor channel:  Front Left  drive motor:        "leftFront"
 * Motor channel:  Front Right drive motor:        "rightFront"
 * Motor channel:  Back Left drive motor:          "leftRear"
 * Motor channel:  Back Right drive motor:         "rightRear"
 * Motor channel:  Linear Slider motor:            "Slider"
 */

public class Hardware6417 {

    //TeleOp variables
    /* public op mode members */
    public DcMotorEx leftFront  = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear   = null;
    public DcMotorEx rightRear  = null;
    public DcMotorEx slider     = null;

    public Servo grabber        = null;

    public BNO055IMU imu;

    Orientation lastAngles      = new Orientation();
    double globalAngle;

    //Autonomous Variables
    SampleMecanumDrive drive;

    /* measurement variables */
    // calculate these in inches
    public double DIAMETER      = ControlConstants.DIAMETER;
    public double CPR           = ControlConstants.CPR;
    public double CIRC          = ControlConstants.CIRC;

    //camera variables
    double[] subMatCenter = {0.45,0.6};
    int subMatWidth = 80;
    int subMatHeight = 100;

    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 360;

    OpenCvWebcam webcam;
    MarkerDetectorPipeline pipeline;

    int position = 0;

    /* Constructor */
    public Hardware6417(){
    }

    HardwareMap hwMap           = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap       = ahwMap;

        // Define and initialize motor and servo
        leftFront   = hwMap.get(DcMotorEx.class, "FrontLeft");
        leftRear    = hwMap.get(DcMotorEx.class, "BackLeft");
        rightFront  = hwMap.get(DcMotorEx.class, "FrontRight");
        rightRear   = hwMap.get(DcMotorEx.class, "BackRight");
        slider      = hwMap.get(DcMotorEx.class, "Slider");

        grabber     = hwMap.get(Servo.class, "Grabber");

        // Set motor and servo directions based on orientation of motors on robot
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        slider.setPower(0);

        //set brake behavior
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //use motor encoders
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target positions first so that error doesn't occur
        rightFront.setTargetPosition(0);
        leftFront.setTargetPosition(0);
        rightRear.setTargetPosition(0);
        leftRear.setTargetPosition(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public void initAutonomous(HardwareMap ahwMap, Telemetry tele){
        //initialize normal hardware
        init(ahwMap);

        //initialize roadrunner
        drive = new SampleMecanumDrive(ahwMap);

        //initialize camera

        //initialize camera
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = ahwMap.get(WebcamName.class, "webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new MarkerDetectorPipeline(subMatCenter[0], subMatCenter[1],subMatWidth,subMatHeight, CAMERA_WIDTH, CAMERA_HEIGHT);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                tele.addData("Camera Failed","");
                tele.update();
            }
        }); //done initializing camera
    }

    /**
     * Robot Methods:
     * start(startGrabPos) - sets grabber to starting position, other necessary
     * setDriveSpeeds(forwardleft, forwardright, strafeleft, straferight) - sets wheel motor speeds to drive
     * holonomicDrive(power, angle) - holonomic drive
     * getAngle() - get angle of robot
     * resetAngle() - resets angle
     * slide(power) - moves slider
     * grab(pos)
     * stop() - stops the robot
     * end() - stops all motors
     */

    public void start(double startGrabPos){
        resetAngle();
        grabber.setPosition(startGrabPos);
    }

    public void setDriveSpeeds(double power, double forwardleft, double forwardright, double strafeleft, double straferight) {


        double leftFrontSpeed = forwardleft + strafeleft;
        double rightFrontSpeed = forwardright - straferight;
        double leftRearSpeed = forwardleft - strafeleft;
        double rightRearSpeed = forwardright + straferight;

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(leftFrontSpeed));
        largest = Math.max(largest, Math.abs(rightFrontSpeed));
        largest = Math.max(largest, Math.abs(leftRearSpeed));
        largest = Math.max(largest, Math.abs(rightRearSpeed));

        leftFront.setPower(power * leftFrontSpeed / largest);
        rightFront.setPower(power * rightFrontSpeed / largest);
        leftRear.setPower(power * leftRearSpeed / largest);
        rightRear.setPower(power * rightRearSpeed / largest);
    }

    public void holonomicDrive(double power, double horizontal, double vertical, double rotation, boolean maintainHeading){
        double magnitude = Math.sqrt((Math.pow(horizontal,2.0)) + (Math.pow(vertical,2.0)));
        double inputAngle = Math.atan2(vertical,horizontal);

        double resultAngle = inputAngle;

        if(maintainHeading){
            double robotAngle = getAngle();
            resultAngle = inputAngle - robotAngle;
        }

        double resultVertical = magnitude * Math.sin(resultAngle);
        double resultHorizontal = magnitude * Math.cos(resultAngle);

        setDriveSpeeds(power, resultVertical + rotation, resultVertical - rotation, resultHorizontal, resultHorizontal);
    }

    public double getAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -Math.PI)
            deltaAngle += 2 * Math.PI;
        else if (deltaAngle > Math.PI)
            deltaAngle -= 2 * Math.PI;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    public void slide(double power){
        slider.setPower(power);
    }

    public void grab(double pos){
        grabber.setPosition(pos);
    }

    public void openGrabber(){
        grabber.setPosition(ControlConstants.grabberOpenPos);
    }

    public void closeGrabber(){
        grabber.setPosition(ControlConstants.grabberClosePos);
    }

    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void end(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        slider.setPower(0);
    }
}
