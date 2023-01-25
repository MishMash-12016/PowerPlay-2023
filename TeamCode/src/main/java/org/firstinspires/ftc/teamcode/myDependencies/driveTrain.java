package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.ElevatorPositions;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Gamepad;
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
    }
    // endregion

    // region FUNCTIONALITY
    /*
    Thread controller = new Thread(() -> {
        isControllerActive = true;
        while(isControllerActive) {
            // make the bot field oriented while considering the starting angle
            joystick_left.addAngle(-getRobotAngle() - AutonomousLeft.lastAngle);


            // using my equations to calculate the power ratios
            DrivingPowerA = (joystick_left.y - joystick_left.x) / sq2 * drivingStrength;
            DrivingPowerB = (joystick_left.y + joystick_left.x) / sq2 * drivingStrength;

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
    // endregion*/

    // region PRIVATE FUNCTIONALITY
    private static void setPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        frontRight.setPower(frontRightPower);
        frontLeft .setPower(frontLeftPower );
        backRight .setPower(backRightPower );
        backLeft  .setPower(backLeftPower  );
    }
    // endregion
}
