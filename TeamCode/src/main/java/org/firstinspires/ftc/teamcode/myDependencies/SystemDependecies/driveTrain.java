package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.util.BNO055IMUUtil;

import java.util.List;

public class driveTrain {
    // region MOTORS
    private static DcMotor frontRight;
    private static DcMotor frontLeft ;
    private static DcMotor backRight ;
    private static DcMotor backLeft  ;
    // endregion

    // region CONSTANTS
    private static final double sq2 = 1.414;

    private static final double fastDrivingStrength = 1  ;
    private static final double fastTurningStrength = 0.8;

    private static final double slowDrivingStrength = 0.4;
    private static final double slowTurningStrength = 0.2;
    // endregion

    // region VARIABLES
    private static double drivingStrength;
    private static double turningStrength;
    public  static double startingAngle;


    private static double x;
    private static double y;
    private static double a;
    private static double l;

    private static double DrivingPowerA;
    private static double DrivingPowerB;
    private static double turningPower;

    private static double normalizingValue;

    private static double frontRightPower;
    private static double frontLeftPower;
    private static double backRightPower;
    private static double backLeftPower;
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

        // region VARIABLES
        drivingStrength = 1;
        turningStrength = 1;
        // endregion

        // region INITIALIZE IMU
        imu = RobotSystem.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        // endregion
    }
    public static void initializeForAuto(BNO055IMU imu, List<DcMotorEx> motors){
        // region MOTORS
        frontLeft  = motors.get(0);
        backLeft   = motors.get(1);
        backRight  = motors.get(2);
        frontRight = motors.get(3);

        // region VARIABLES
        drivingStrength = 1;
        turningStrength = 1;
        // endregion

        driveTrain.imu = imu;
    }

    public static void reset(){
        drivingStrength = 1;
        turningStrength = 1;
        startingAngle = 0;
    }
    // endregion

    // region FUNCTIONALITY
    public static Thread controller = new Thread(() -> {
        while(!RobotSystem.isStopRequested && !driveTrain.controller.isInterrupted()) {
            // region GET THE ORIGINAL DIRECTION INPUT
            x =  RobotSystem.gamepad1.left_stick_x;
            y = -RobotSystem.gamepad1.left_stick_y;
            // endregion

            // region MAKE IT FIELD ORIENTED
            a = Math.atan2(x, y);
            a += Math.PI * 2 - getRobotAngle();
            a = a % (Math.PI * 2);

            l = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            x = Math.sin(a) * l;
            y = Math.cos(a) * l;
            // endregion

            // region CALCULATE THE POWER
            DrivingPowerA = (y - x) / sq2 * drivingStrength;
            DrivingPowerB = (y + x) / sq2 * drivingStrength;

            turningPower = RobotSystem.gamepad1.right_stick_x * turningStrength;

            frontRightPower = DrivingPowerA - turningPower;
            frontLeftPower  = DrivingPowerB + turningPower;
            backRightPower  = DrivingPowerB - turningPower;
            backLeftPower   = DrivingPowerA + turningPower;
            // endregion

            // region NORMALIZE THE POWER
            normalizingValue = Math.max(1, Math.max(Math.max(Math.abs(
                    frontRightPower
            ), Math.abs(
                    frontLeftPower
            )), Math.max(Math.abs(
                    backRightPower
            ), Math.abs(
                    backLeftPower
            ))));

            frontRightPower /= normalizingValue;
            frontLeftPower  /= normalizingValue;
            backRightPower  /= normalizingValue;
            backLeftPower   /= normalizingValue;
            // endregion

            // region SET THE NORMALIZED POWER
            setPower(frontRightPower,
                     frontLeftPower ,
                     backRightPower ,
                     backLeftPower  );
            // endregion
        }
    });
    public static void slowMode(){
        drivingStrength = slowDrivingStrength;
        turningStrength = slowTurningStrength;
    }
    public static void fastMode(){
        drivingStrength = fastDrivingStrength;
        turningStrength = fastTurningStrength;
    }

    public static void resetFieldOriented(){
        startingAngle = 0;
        startingAngle = -getRobotAngle();
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    public static void setPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        frontRight.setPower(frontRightPower);
        frontLeft .setPower(frontLeftPower );
        backRight .setPower(backRightPower );
        backLeft  .setPower(backLeftPower  );
    }

    private static BNO055IMU imu;
    public static double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) {
            angle += Math.PI * 2;
        }
        angle = Math.PI * 2 - angle;

        return angle + driveTrain.startingAngle;
    }
    // endregion
}
