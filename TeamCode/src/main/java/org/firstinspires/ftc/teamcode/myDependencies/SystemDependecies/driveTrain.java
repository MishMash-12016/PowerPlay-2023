package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

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
    public  static double  startingAngle;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region MOTORS
        frontRight = RobotSystem.hardwareMap.dcMotor.get("frontRight");
        frontLeft  = RobotSystem.hardwareMap.dcMotor.get("frontLeft" );
        backRight  = RobotSystem.hardwareMap.dcMotor.get("backRight" );
        backLeft   = RobotSystem.hardwareMap.dcMotor.get("backLeft"  );

        // flipping the two left driving motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft .setDirection(DcMotorSimple.Direction.REVERSE);
        // endregion

        // region SENSORS
        imu = RobotSystem.hardwareMap.get(BNO055IMU.class, "imu");

        // initializing the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        // endregion
    }
    public static void reset(){
        drivingStrength = 1;
        turningStrength = 1;
        startingAngle = 0;
    }
    // endregion

    // region FUNCTIONALITY
    public static Thread controller = new Thread(() -> {
        double x;
        double y;
        double a;
        double l;

        double DrivingPowerA;
        double DrivingPowerB;
        double turningPower;

        while(!RobotSystem.isStopRequested && !driveTrain.controller.isInterrupted()) {
            // region GET THE ORIGINAL DIRECTION INPUT
            x =  RobotSystem.gamepad1.left_stick_x;
            y = -RobotSystem.gamepad1.left_stick_y;
            // endregion

            // region MAKE IT FIELD ORIENTED
            a = Math.atan2(x, y);
            a += Math.PI * 2 - getRobotAngle() - startingAngle;
            a = a % (Math.PI * 2);

            l = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            x = Math.sin(a) * l;
            y = Math.cos(a) * l;
            // endregion

            // region CALCULATE AND SET THE POWER
            DrivingPowerA = (y - x) / sq2 * drivingStrength;
            DrivingPowerB = (y + x) / sq2 * drivingStrength;

            turningPower = RobotSystem.gamepad1.right_stick_x * turningStrength;

            // setting the powers in consideration of the turning speed
            setPower(DrivingPowerA - turningPower,
                    DrivingPowerB + turningPower,
                    DrivingPowerB - turningPower,
                    DrivingPowerA + turningPower);
            // endregion
        }

        driveTrain.reset();
    });
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        frontRight.setPower(frontRightPower);
        frontLeft .setPower(frontLeftPower );
        backRight .setPower(backRightPower );
        backLeft  .setPower(backLeftPower  );
    }
    private static double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) {
            angle += Math.PI * 2;
        }
        angle = Math.PI * 2 - angle;

        return angle;
    }
    // endregion
}
