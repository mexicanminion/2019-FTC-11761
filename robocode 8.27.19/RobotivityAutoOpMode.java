package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.vuforia.CameraDevice;


@Autonomous(name = "Robotivity Auto Mode", group = "Robotivity")
public class RobotivityAutoOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AbXRmTn/////AAABmcWLKN1CR0y+nV5whIiA56NspFSu58qvwso6A0B0Tt0t0S6ueSYPk0OXhBjbDzK8jwJh4N2kf5i1w+eEm6KmFS8Gl5cuzZ963QVbVUf+Xksab1/Eg5/gdFQOkMszNPZMcMrglQCQjUjakm6Q6uEmgGTMhjxS3DjLaCm5SowAEoZk4YC8p+JnPL4/EN7o/+pluwCBZ8KyPNd9m5zzY8Odin31ll8mH/3yeNyn9hK5GiPF9eaG2iClcmK/5D+J1sGrnuY9iD3k36N3awWTgVhzhGVAz84I+kBOgsc1SFOgJpWJKSHJFSB4SmE6gXTjMJIPc3xwFItRx/OwskGjk6EOgKV7T9ZnqXxjB1Ye7fMdmti9";
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
//Motors
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor armMotor = null;
    private Servo tokenServo = null;
//Robot Movement
    private int ElementTurnAngle = 29;
    private double pi = 3.141;
    private double RobotWidth = 2.54 * 15.5;
    //distances were subtracted by 12
    private int ElementPushDistance = 65;
    private int DepotStretchDistance1 = 110;
    private int DepotStretchDistance2 = 128;
    private int DepotStretchDistance3 = 103;
    private int DepotStretchDistance4 = 78;
    private int CraterStretchDistance = 213;
    private int ForwardDriveDetach = 5;
    private int elementSeekAngle = 25;
    private int elementSeekAngle2 = 45;
    private int goldHeightThreshole = 1000;


    public void TensorInt() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }





  /*
  not used

   public void FaceGold(int Direction) {
        if (Direction == 1) {
            //  DoTurnLeft(ElementTurnAngle);

        } else if (Direction == 2) {
        }// it's going striaght forward
        else if (Direction == 3) {
            DoTurnRight(ElementTurnAngle);
        }
    }*/

    /*
  not used
    public void DoTurnRight(int Angle){
        //   int DistanceToDrive;
        //   DistanceToDrive = (int) (1.35*pi * RobotWidth * Angle / 360);
        //   TurnMotorsForward(DistanceToDrive, .7, -1);
//        TurnAnlgeGyro((double)-Angle);
}*/


    /*
not used
    public double GetGyroError() {
        return 0;
    }*/


    /*
not used
    public int HeadingError(int Current, int Target) {
        int error;

        error = Current - Target;

        if (error > 180)
            error = error - 360;
        return error;
    }*/


    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private int robotHeading = 0;


    //private double ticksPerRev = 1200;
    private double ticksPerRev = 288;
    //private double wheelDiameter = 10;//diameter in cm
    private double wheelDiameter = 9;
    private double distanceFactor = ticksPerRev / (wheelDiameter * pi);


    public void initialize() {
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        robotHeading = 0;
        modernRoboticsI2cGyro.calibrate();
        while (modernRoboticsI2cGyro.isCalibrating()) {
//            telemetry.addData(">", "Calibrating...");
//            telemetry.update();
            sleep(50);
        }
        ;

        //Initialize motors, servos etc...
        //e.g.
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        armMotor = hardwareMap.get(DcMotor.class,"Arm motor");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");

    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Drive forward/backwards in the current tracked direction
    //Note, the heading (i.e. direction) will be the sum of all the turns we
    //have made (i.e. where we SHOULD be pointing) and NOT the direction the
    //robot was pointing when we started to drive. This should therefore
    //account for any changes caused by bumping, colliding etc...
    public void driveStraight(int distance, double speed) {
        driveHeading(distance, robotHeading, speed);
    }

    //Drive forward/backwards in the currently faced direction
    //This will not account for unknown changes in rotation
    public void driveLine(int distance, double speed) {
        int currentHeading;

        currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
        driveHeading(distance, currentHeading, speed);
    }

    //Drive distance with power setting speed, turning to face the specified heading
    //Negative distances will drive backwards
    //When backwards it will still be the FRONT of the robot facing the heading
    //Note, speed can be +ve or -ve and will be automatically corrected
    public void driveHeading(int distance, int targetHeading, double speed) {
        int error;
        int currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;
        double gain = .06;
        int distanceTraveled;

        resetEncoders();

        if (distance > 0) {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        } else {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }

        do {
            //Find where we are currently pointing
            currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();

            //Calculate how far off we are
            error = (currentHeading - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speed + speedCorrection;
            rightSpeed = speed - speedCorrection;

            //Apply the power settings to the motors
            frontLeftMotor.setPower(leftSpeed);
            backLeftMotor.setPower(leftSpeed);
            frontRightMotor.setPower(rightSpeed);
            backRightMotor.setPower(rightSpeed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (int) (((frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / (distanceFactor * 4)));


            telemetry.addData("frontLeftMotorPosition", frontLeftMotor.getCurrentPosition());
            telemetry.update();

        }
        while ((Math.abs(distanceTraveled) < Math.abs(distance)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }

    //Turn the robot by the specified angle regardless of errors
    //Note, this will turn from the current heading and not the tracked
    //heading
    public void rotateAnlgeGyro(int angleToTurn, double speed) {
        int targetHeading;
        int currentHeading;

        currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();

        targetHeading = currentHeading + angleToTurn;

        turnHeadingGyro(targetHeading, speed);
    }


    //Turn the robot by the specified angle
    //Note, this will turn to the tracked position so will account for
    //inflicted errors in direction
    public void turnAnlgeGyro(int angleToTurn, double speed) {
        int targetHeading;

        targetHeading = robotHeading + angleToTurn;

        turnHeadingGyro(targetHeading, speed);
    }

    //Turn to face the specified heading.
    //This will account for inflicted errors in direction
    public void turnHeadingGyro(int targetHeading, double speed) {
        int error;
        int errorThreshold = 2;//How accurate do we need to be? Too high, wrong direction, too small will oscillate
        int currentHeading;
        double turnSpeed;
        double motorPower;

        do {
            currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
            error = (currentHeading - targetHeading);
            if (error == 0) {
                //Just on the offchance we are actually at an error of zero
                //stop motors ASAP
                turnSpeed = 0.0;
            }
            if (Math.abs(error) > 5) {
                //If need to turn a lot then go at full speed
                turnSpeed = speed;
            } else {
                //Close to correct heading so slow down
                turnSpeed = speed * 0.3;
            }

            if (error > 0) {
                //Positive error so need to turn clockwise
                motorPower = -turnSpeed;
            } else {
                //Negative error so need to turn anti-clockwise
                motorPower = turnSpeed;
            }

            //Turn on the motors
            frontLeftMotor.setPower(-motorPower);
            backLeftMotor.setPower(-motorPower);
            frontRightMotor.setPower(motorPower);
            backRightMotor.setPower(motorPower);

//            telemetry.addData(">", "e = %d h = %d p = %0.2f",error, currentHeading, motorPower);
//            telemetry.update();
        }
        while ((Math.abs(error) > errorThreshold) && opModeIsActive());//Keep turning until close enough to correct heading

        //Turn off the motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }


    public boolean IsGoldAhead() {
        int goldMineralX = -1;
        //int silverMineralX = -1;
        boolean Elementfound = false;
        while (opModeIsActive() && (!Elementfound)) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                CameraDevice.getInstance().setFlashTorchMode(true);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && (recognition.getBottom() < goldHeightThreshole)) {
                            goldMineralX = (int) recognition.getLeft();
                           // telemetry.addData("Y value",);
                        } //else {
                            //silverMineralX = (int) recognition.getLeft();
                        //}
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL) || recognition.getLabel().equals(LABEL_SILVER_MINERAL))Elementfound = true;
                    }

                }
            }
             //else if (silverMineralX != -1) {
                //return false;
           // }

        }
        if (goldMineralX != -1) {
            CameraDevice.getInstance().setFlashTorchMode(false);
            return true;
        }
        return false;
    }

    public int LookForGold() {
        sleep(500);
        if (IsGoldAhead()) {
            return 2;
        } else {
            turnAnlgeGyro(elementSeekAngle, 0.6);
            sleep(500);
            if (IsGoldAhead()) {
                return 1;
            } else {
                turnAnlgeGyro(-(elementSeekAngle * 2), 0.6);
                return 3;
            }
        }
    }

    public void PushGold(int postition) {
        if (postition==1)
        driveHeading(ElementPushDistance+5, elementSeekAngle,1);
        else if (postition==2)
            driveHeading(ElementPushDistance,0,1);
        else
            driveHeading(ElementPushDistance+5,-(elementSeekAngle2 - 20),1);

    }

    public void ReturnStart(int position) {
        if (position==1) {
            driveHeading(-ElementPushDistance - 5, elementSeekAngle, 1);
        }
        else if (position==2) {
            driveHeading(-ElementPushDistance, 0, 1);
        }
        else
            driveHeading(-ElementPushDistance-5,-(elementSeekAngle2 - 20),1);
            //turnAnlgeGyro(40,1);


    }


    public void SampleElement() {
        int Position;
        //Look for gold
        Position = LookForGold();
        //Push the gold
        PushGold(Position);
        //return to start
        ReturnStart(Position);
    }


    public void DepositToken1() {
            turnAnlgeGyro(43, 1);
            driveHeading(DepotStretchDistance1, 43, 1);
            turnAnlgeGyro(85, 1);
            driveHeading(DepotStretchDistance2, 135, 1);
            tokenServo.setPosition(0.5);
            sleep(2000);
    }


    public void DepositToken2_old() {
        //turnAnlgeGyro(-43,1);
        turnHeadingGyro(-43,1);
        driveHeading(DepotStretchDistance3, -43, 1);
        turnAnlgeGyro(83, 1);
        driveHeading(DepotStretchDistance4, 45, 1);
        tokenServo.setPosition(0.5);
        sleep(2000);
    }
    public void DepositToken2(){
        turnHeadingGyro(90,1);
        driveHeading(-5,90,1);
        turnHeadingGyro(135,1);
        driveHeading(-103,135,1);
        turnHeadingGyro(45,1);
        driveHeading(78,45,1);
        tokenServo.setPosition(0.5);
        sleep(2000);
    }


    public void ParkInCrater1() {
        driveHeading(-CraterStretchDistance, 135, 1);
    }


    public void ParkInCrater2() {

        driveHeading(-CraterStretchDistance, 45, 1);
    }

    public void lowerRobot(){
        armMotor.setPower(-1);
        sleep(3000);
        turnAnlgeGyro(-45,1);
        driveHeading(ForwardDriveDetach,-45,1);
        turnAnlgeGyro(45,1);

    }


    public void ResetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    public void runOpMode() throws InterruptedException {

        boolean hasRun = false;
        // Initialize variables here
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        //armMotor = hardwareMap.get(DcMotor.class,"Arm motor");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        TensorInt();



        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        runtime.reset();


        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        modernRoboticsI2cGyro.calibrate();
        while (modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData(">", "Calibrating...");
            telemetry.update();
            sleep(50);
        }
        ;
        telemetry.addData(">", "Done...");
        telemetry.update();

        // Crater();
        Depot();
        // driveHeading(30,0,1);
        //tokenServo.setPosition(0.3);
        //driveForward(100, 1);

    }

    private void Crater() {
        //lowerRobot();
        SampleElement();
        DepositToken1();
        ParkInCrater1();
    }

    private void Depot() {
        //lowerRobot();
        SampleElement();
        DepositToken2_old();
        ParkInCrater2();
    }


    private void driveForward(int distanceInCentimeters, double speed) {
        TurnMotorsForward(distanceInCentimeters, speed, 1);
    }


    private void TurnMotorsForward(int distanceInCentimeters, double speed, int Mode) {
        // Calculate distance based on a conversion from motor position
        int desiredPosition = (int) (distanceInCentimeters * 27.41);

        ResetEncoders();

        while (frontLeftMotor.getCurrentPosition() < desiredPosition) {
            telemetry.addData("frontLeftMotorPosition", frontLeftMotor.getCurrentPosition());
            // Drive the robot forward
            frontLeftMotor.setPower(speed);
            frontRightMotor.setPower(speed * Mode);
            backRightMotor.setPower(speed * Mode);
            backLeftMotor.setPower(speed);

        }
        // Stop the robot
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


    private void driveBackward(int distanceInCentimeters, double speed) {
        TurnMotorsBackward(distanceInCentimeters, speed, 1);
    }

    private void TurnMotorsBackward(int distanceInCentimeters, double speed, int Mode) {
        // Calculate distance based on a conversion from motor position
        int desiredPosition = (int) (distanceInCentimeters * 27.41);

        ResetEncoders();
        // While we haven't reach the position
        // While current distance travelled is less than desired distance
        while (frontLeftMotor.getCurrentPosition() > desiredPosition) {
            telemetry.addData("frontLeftMotorPosition", frontLeftMotor.getCurrentPosition());
            // Drive the robot forward
            frontLeftMotor.setPower(-speed);
            frontRightMotor.setPower(-speed * Mode);
            backRightMotor.setPower(-speed * Mode);
            backLeftMotor.setPower(-speed);
        }
        // Stop the robot
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}
