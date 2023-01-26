package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.ElevatorPositions;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Gamepad;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Vector;
import org.firstinspires.ftc.teamcode.opModes.autonomousOpModes.AutonomousLeft;

public class driveTrain {
    // region MOTORS
    private static DcMotor frontRight;
    private static DcMotor frontLeft ;
    private static DcMotor backRight ;
    private static DcMotor backLeft  ;
    // endregion

    // region SENSORS
    private static BNO055IMU imu;
    // endregion

    // region CONSTANTS
    private static final double sq2 = 1.414;
    // endregion

    // region VARIABLES
    private static double drivingStrength;
    private static double turningStrength;
    private static System.DriveMode mode;
    private static boolean isControllerActive;
    private static boolean startingAngle;

    private static double DrivingPowerA;
    private static double DrivingPowerB;
    private static double turningPower;
    private static Vector joystickLeft;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region MOTORS
        frontRight = System.hardwareMap.dcMotor.get("frontRight");
        frontLeft  = System.hardwareMap.dcMotor.get("frontLeft" );
        backRight  = System.hardwareMap.dcMotor.get("backRight" );
        backLeft   = System.hardwareMap.dcMotor.get("backLeft"  );

        // flipping the two left driving motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        // endregion

        // region SENSORS
        imu = System.hardwareMap.get(BNO055IMU.class, "imu");

        // initializing the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        // endregion

        reset();
    }
    public static void reset(){
        drivingStrength = 1;
        turningStrength = 1;
        mode = null;
        isControllerActive = false;

        DrivingPowerA = 0;
        DrivingPowerB = 0;
        turningPower = 0;
        joystickLeft = new Vector();
    }
    // endregion

    // region FUNCTIONALITY

    Thread controller = new Thread(() -> {
        isControllerActive = true;
        while(isControllerActive) {
            joystickLeft.x = System.gamepad1.left_stick_x;
            joystickLeft.y = System.gamepad1.left_stick_y;

            // make the bot field oriented while considering the starting angle
            joystickLeft.addAngle(-getRobotAngle() - AutonomousLeft.lastAngle);


            // using my equations to calculate the power ratios
            DrivingPowerA = (joystickLeft.getY() - joystickLeft.getX()) / sq2 * drivingStrength;
            DrivingPowerB = (joystickLeft.getY() + joystickLeft.getX()) / sq2 * drivingStrength;

            turningPower = Gamepad.right_stick_x * turningStrength;

            // setting the powers in consideration of the turning speed
            setPower(DrivingPowerA - turningPower,
                    DrivingPowerB + turningPower,
                    DrivingPowerB - turningPower,
                    DrivingPowerA + turningPower);
        }
    });
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        frontRight.setPower(frontRightPower);
        frontLeft .setPower(frontLeftPower );
        backRight .setPower(backRightPower );
        backLeft  .setPower(backLeftPower  );
    }
    private static void calculateAndSetDrivingPower(double x, double y){

    }
    // endregion
}
