package org.firstinspires.ftc.teamcode.myDependencies.oldFiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.opModes.autonomousOpModes.AutonomousLeft;

public class RobotController {

    private ElapsedTime et = new ElapsedTime();
    private ElapsedTime grabberSafety = new ElapsedTime();
    private ElapsedTime pufferSafety = new ElapsedTime();
    private ElapsedTime safety = new ElapsedTime();
    /** GENERAL CONSTANTS */
    private final double sq2 = Math.sqrt(2);
    private final Telemetry telemetry;

    /** THREADS */
    public final Thread elevatorController;
    public final Thread driveController;
    public final Thread cycleController;

    private final Thread teleScore;
    public final Thread autoCycle;

    /** SERVO CONSTANTS */
    private final double grabberMiddle = 0.26;
    public final double grabberIn  = 0.09;

    //                                  low > > > > > > > > > > > high
    public final double[] grabberPile = {0.76, 0.72, 0.69, 0.65, 0.61};

    private final double armOut = 0.89; // hiTech value 0.61;
    private final double armIn  = 0.5;  // hiTech value 0.41;

    private final double placerIn  = 0;
    private final double placerOut = 0.75;// 0.78

    private final double pufferGrab    = 0.14;
    private final double pufferRelease = 0;

    private final double grabberGrab = 0.61;
    private final double grabberOpen = 0.36;

    /** SENSOR CONSTANTS */
    private final double grabberCatchTrigger = 8.5;

    /** SENSORS */
    private final BNO055IMU imu;
    private final DistanceSensor grabberDistanceSensor;
    private final DigitalChannel armSensor;
    //private final DigitalChannel elevatorSensor;
    private final DigitalChannel grabberPositionSensor;

    /** SERVOS */
    private final Servo puffer ;
    private final Servo grabber;

    private final Servo placerRight;
    private final Servo placerLeft ;

    private final Servo armRight;
    private final Servo armLeft ;

    private final Servo grabberRight;
    private final Servo grabberLeft ;

    /** DRIVING MOTORS */
    private final DcMotor frontRight;
    private final DcMotor frontLeft ;
    private final DcMotor backRight ;
    private final DcMotor backLeft  ;

    /** DRIVING STATE KEEPERS */
    public double overallDrivingPower;
    public double overallTurningPower;

    /** ELEVATOR MOTORS */
    private final DcMotorEx elevatorLeft ;
    private final DcMotorEx elevatorRight;

    /** ELEVATOR STATE KEEPERS */
    int elevatorPosition;
    double elevatorPower;

    /** DRIVE CONTROLLER */
    Vector joystick_left;
    double DrivingPowerA, DrivingPowerB;
    double turningPower;


    public RobotController(HardwareMap hm, Telemetry t) {
        this.telemetry = t;
        /** SENSORS */
        // getting the imu from the hardware map
        imu = hm.get(BNO055IMU.class, "imu");

        // initializing the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        // getting the grabber sensor
        grabberDistanceSensor = hm.get(DistanceSensor.class, "grabberDistanceSensor");
        // getting the arm sensor
        armSensor = hm.get(DigitalChannel.class, "armSensor");
        armSensor.setMode(DigitalChannel.Mode.INPUT);

        //elevatorSensor = hm.get(DigitalChannel.class, "elevatorSensor");
        //elevatorSensor.setMode(DigitalChannel.Mode.INPUT);

        grabberPositionSensor = hm.get(DigitalChannel.class, "grabberPositionSensor");
        grabberPositionSensor.setMode(DigitalChannel.Mode.INPUT);

        /** SERVOS */
        // getting the edge units
        puffer  = hm.servo.get("puffer" );
        grabber = hm.servo.get("grabber");

        // setting the edge units to their initial positions
        grabber.setPosition(grabberOpen);
        puffer .setPosition(pufferRelease);


        // getting the placers, arms and grabbers
        placerRight = hm.servo.get("placerRight");
        placerLeft  = hm.servo.get("placerLeft" );

        armRight = hm.servo.get("armRight");
        armLeft  = hm.servo.get("armLeft" );

        grabberRight = hm.servo.get("grabberRight");
        grabberLeft  = hm.servo.get("grabberLeft" );

        // flipping the relevant servos direction
        armLeft    .setDirection(Servo.Direction.REVERSE);
        placerRight.setDirection(Servo.Direction.REVERSE);
        grabberLeft.setDirection(Servo.Direction.REVERSE);

        // setting the servos position to their initial positions
        setArmPosition(armIn);
        setPlacerPosition(placerIn);
        setGrabberPosition(grabberIn);


        /** MOTORS */
        // getting the driving motors
        frontRight = hm.dcMotor.get("frontRight");
        frontLeft  = hm.dcMotor.get("frontLeft" );
        backRight  = hm.dcMotor.get("backRight" );
        backLeft   = hm.dcMotor.get("backLeft"  );

        // flipping the two left driving motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft .setDirection(DcMotorSimple.Direction.REVERSE);

        overallDrivingPower = 1;
        overallTurningPower = 1;

        // getting the elevator motors
        elevatorLeft  = (DcMotorEx) hm.dcMotor.get("elevatorLeft" );
        elevatorRight = (DcMotorEx) hm.dcMotor.get("elevatorRight");

        // flipping the right elevator motor
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the left elevator motor to use the encoder
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        teleScore = new Thread(() -> {
            if (elevatorPosition != ElevatorPositions.bottom) {
                try {
                    puffer.setPosition(pufferGrab);
                    grabber.setPosition(grabberOpen);

                    while(elevatorPosition - elevatorLeft.getCurrentPosition() > 10000){
                        if (Gamepad.isStopRequested){
                            throw new InterruptedException("stop requested");
                        }
                    }

                    setPlacerPosition(placerOut);

                    while (Gamepad.right_trigger == 0) {
                        if (Gamepad.isStopRequested) {
                            throw new InterruptedException("stop requested");
                        }
                        telemetry.addData("aaa", "aaa");
                        telemetry.update();
                    }

                    puffer.setPosition(pufferRelease);

                    while (Gamepad.right_trigger > 0) {
                        if (Gamepad.isStopRequested) {
                            throw new InterruptedException("stop requested");
                        }
                        telemetry.addData("aaa", "aaa");
                        telemetry.update();
                    }

                    setPlacerPosition(placerIn);
                    elevatorPosition = ElevatorPositions.bottom;

                } catch (InterruptedException e) {
                }
            } else {
            }
        });

        autoCycle = new Thread(() -> {
            autoScore(true, 4);
            for (int i = 4; i > 0; i--){
                collect(i);
                autoScore(true, i - 1);
            }

            collect(0);
            autoScore(false, 0);
        });


        cycleController = new Thread(() -> {
            Gamepad.isCycleControllerActive = true;
            while (Gamepad.isCycleControllerActive) {
                if (Gamepad.left_trigger > 0 || Gamepad.left_bumper) {

                    // bring the grabber down
                    setGrabberPosition(grabberPile[0]);

                    // bring the grabber out the correct amount
                    setArmPosition(Gamepad.left_trigger * (armOut - armIn) + armIn);

                    // catch the cone if its in range
                    if (grabberDistanceSensor.getDistance(DistanceUnit.CM) < grabberCatchTrigger && grabberSafety.milliseconds() > 500) {
                        grabber.setPosition(grabberGrab);
                    } else {
                        grabber.setPosition(grabberOpen);
                    }
                    pufferSafety.reset();
                    if (!teleScore.isAlive()){
                        puffer.setPosition(pufferRelease);
                    }
                } else {
                    if (!armSensor.getState()) {
                        setGrabberPosition(grabberIn);
                    } else {
                        setArmPosition(armIn);
                    }
                    grabberSafety.reset();
                    if (puffer.getPosition() != pufferGrab && pufferSafety.milliseconds() > 1000 && !teleScore.isAlive()){
                        puffer.setPosition(pufferGrab);
                        grabber.setPosition(grabberOpen);
                    }
                }

                if (A_pressed()) {
                    elevatorPosition = ElevatorPositions.high;
                    if (!teleScore.isAlive()) teleScore.start();
                } else if (X_pressed()) {
                    elevatorPosition = ElevatorPositions.middle;
                    if (!teleScore.isAlive()) teleScore.start();
                } else if (Y_pressed()) {
                    elevatorPosition = ElevatorPositions.low;
                    if (!teleScore.isAlive()) teleScore.start();
                }
            }
        });

        elevatorPosition = ElevatorPositions.bottom;
        elevatorPower = 0;
        elevatorController = new Thread(() -> {
            Gamepad.isElevatorControllerActive = true;
            while (Gamepad.isElevatorControllerActive){
                elevatorPower = Math.max(Math.min((elevatorPosition - elevatorLeft.getCurrentPosition()) / 2300.0, 1), -1);

                // make the elevator weaker when coming down to compensate for gravity
                if (elevatorPower < 0) elevatorPower /= 2;

                // set the elevator power into the elevator motors
                elevatorLeft .setPower(elevatorPower);
                elevatorRight.setPower(elevatorPower);
            }
        });

        joystick_left = new Vector(0, 0);
        driveController = new Thread(() ->{
            Gamepad.isDriveControllerActive = true;
            while(Gamepad.isDriveControllerActive) {
                joystick_left.x = Gamepad.left_stick_x;
                joystick_left.y = -Gamepad.left_stick_y;

                // make the bot field oriented while considering the starting angle
                joystick_left.addAngle(-getRobotAngle() - AutonomousLeft.lastAngle);


                // using my equations to calculate the power ratios
                DrivingPowerA = (joystick_left.y - joystick_left.x) / sq2 * overallDrivingPower;
                DrivingPowerB = (joystick_left.y + joystick_left.x) / sq2 * overallDrivingPower;

                turningPower = Gamepad.right_stick_x * overallTurningPower;

                // slow mode;
                if (grabberLeft.getPosition() == grabberPile[0] ||
                        elevatorPosition != ElevatorPositions.bottom ||
                        Gamepad.right_bumper) {
                    overallDrivingPower = 0.4;
                    overallTurningPower = 0.2;
                } else {
                    overallDrivingPower = 1;
                    overallTurningPower = 1;
                }
                // setting the powers in consideration of the turning speed
                setDrivingPower(DrivingPowerA - turningPower,
                        DrivingPowerB + turningPower,
                        DrivingPowerB - turningPower,
                        DrivingPowerA + turningPower);
            }
        });
    }

    public void safeSleep(int millisecond) throws InterruptedException {
        et.reset();
        while (et.milliseconds() < millisecond){ if (Gamepad.isStopRequested) throw new InterruptedException("stop requested"); }
    }

    private boolean a = true;
    private boolean A_pressed(){
        if (Gamepad.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }

    private boolean x = true;
    private boolean X_pressed(){
        if (Gamepad.x){
            if(x){
                x = false;
                return true;
            }
        } else x = true;
        return false;
    }

    private boolean y = true;
    private boolean Y_pressed(){
        if (Gamepad.y){
            if(y){
                y = false;
                return true;
            }
        } else y = true;
        return false;
    }

    public double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) angle += Math.PI * 2;
        angle = Math.PI * 2 - angle;

        return angle;
    }

    public void setArmPosition(double position) {
        armLeft .setPosition(position);
        armRight.setPosition(position);
    }

    public void setPlacerPosition(double position) {
        placerLeft .setPosition(position);
        placerRight.setPosition(position);
    }

    public void setGrabberPosition(double position) {
        grabberLeft .setPosition(position);
        grabberRight.setPosition(position);
    }

    public void setDrivingPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        frontRight.setPower(frontRightPower);
        frontLeft .setPower(frontLeftPower );
        backRight .setPower(backRightPower );
        backLeft  .setPower(backLeftPower  );
    }

    public void autoScore(boolean prepareForNext, int nextConeNum){
        try {
            puffer.setPosition(pufferGrab);
            if (grabber.getPosition() != grabberOpen){
                grabber.setPosition(grabberOpen);
                safeSleep(100);
            }
            elevatorPosition = ElevatorPositions.high;

            safeSleep(750);

            setPlacerPosition(placerOut);

            if (prepareForNext) {
                setGrabberPosition(grabberPile[nextConeNum]);

                setArmPosition(armOut * 0.8);

            }
            safeSleep(500);

            puffer.setPosition(pufferRelease);

            safeSleep(400);

            setPlacerPosition(placerIn);
            elevatorPosition = ElevatorPositions.bottom;
        }catch (InterruptedException e){}

    }

    public void collect(int coneNum){
        try {
            setGrabberPosition(grabberPile[coneNum]);
            safeSleep((int)(500 / (grabberIn - grabberPile[0]) * Math.abs(grabberPile[coneNum] - grabberLeft.getPosition())));

            setArmPosition(armOut);

            safeSleep(400);

//            while (grabberDistanceSensor.getDistance(DistanceUnit.CM) < grabberCatchTrigger) {
//                if (gamepad.isStopRequested) throw new InterruptedException("stop requested");
//            }

            grabber.setPosition(grabberGrab);

            safeSleep(400);

            setGrabberPosition(grabberMiddle);

            safeSleep(500);

            setArmPosition(armIn);

            safety.reset();
            while (armSensor.getState() && safety.milliseconds() < 600) {
                if (Gamepad.isStopRequested) throw new InterruptedException("stop requested");
            }

            setGrabberPosition(grabberIn);

            safety.reset();
            while (grabberPositionSensor.getState() && safety.milliseconds() < 500){
                if (Gamepad.isStopRequested) throw new InterruptedException("stop requested");
            }
        }catch (InterruptedException e){}
    }

    public void terminate(){
        elevatorController.interrupt();
        driveController.interrupt();
        cycleController.interrupt();
        teleScore.interrupt();
        autoCycle.interrupt();

        elevatorPosition = ElevatorPositions.bottom;

        Gamepad.a = false;
        Gamepad.b = false;
        Gamepad.y = false;
        Gamepad.x = false;

        Gamepad.left_trigger = 0;
        Gamepad.right_trigger = 0;

        Gamepad.left_bumper = false;
        Gamepad.right_bumper = false;

        Gamepad.left_stick_y = 0;
        Gamepad.left_stick_x = 0;
        Gamepad.right_stick_x = 0;

        Gamepad.isDriveControllerActive = false;
        Gamepad.isElevatorControllerActive = false;
        Gamepad.isCycleControllerActive = false;


    }

}