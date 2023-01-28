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
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Vector;

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
    private static RobotSystem.DriveMode mode;
    private static boolean isControllerActive;
    public  static double  startingAngle;

    private static double DrivingPowerA;
    private static double DrivingPowerB;
    private static double turningPower;
    private static Vector joystickLeft;
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

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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
        mode = null;
        isControllerActive = false;
        startingAngle = 0;

        DrivingPowerA = 0;
        DrivingPowerB = 0;
        turningPower = 0;
        joystickLeft = new Vector();
    }
    // endregion

    // region FUNCTIONALITY
    public static Thread controller = new Thread(() -> {
        isControllerActive = true;
        double x;
        double y;
        double a;
        double l;
        while(isControllerActive) {
            x = RobotSystem.gamepad1.left_stick_x;
            y = RobotSystem.gamepad1.left_stick_y;
            a = Math.PI * 2;
            if (y < 0) a += Math.PI + Math.atan(-x / y);
            else if (y == 0) {
                if (x < 0) a += Math.PI * 0.5;
                else a += Math.PI * 1.5;
            } else {
                a += Math.atan(-x / y);
            }
            a = Math.PI * 2 - a % (Math.PI * 2);

            a += -getRobotAngle() - startingAngle;

            l = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            x = Math.sin(a) * l;
            y = Math.cos(a) * l;

            DrivingPowerA = (y - x) / sq2 * drivingStrength;
            DrivingPowerB = (y + x) / sq2 * drivingStrength;

            turningPower = RobotSystem.gamepad1.right_stick_x * turningStrength;

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
