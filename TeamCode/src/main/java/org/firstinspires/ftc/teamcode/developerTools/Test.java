package org.firstinspires.ftc.teamcode.developerTools;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.elevator;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@TeleOp
public class Test extends LinearOpMode {
    static DcMotor frontRight;
    static DcMotor frontLeft;
    static DcMotor backRight;
    static DcMotor backLeft;
    @Override
    public void runOpMode(){
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft  = hardwareMap.dcMotor.get("frontLeft" );
        backRight  = hardwareMap.dcMotor.get("backRight" );
        backLeft   = hardwareMap.dcMotor.get("backLeft"  );

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // region INITIALIZE IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        // endregion

        waitForStart();

        while (opModeIsActive()){
            setPower(f((getRobotAngle() - Math.PI) / Math.PI));
            telemetry.addData("angle", getRobotAngle() - Math.PI);
            telemetry.update();
        }
    }
    static double marginOfError = 0.015;
    static double smoothness = 0.005;
    static double maxTurning = 0.3;
    public static double f(double x){

        if (Math.abs(x) < marginOfError){
            return 0;
        } else if (x > smoothness + marginOfError){
            return -maxTurning;
        } else if (-x > smoothness + marginOfError){
            return maxTurning;
        }

        if (x > 0){
            x -= marginOfError;
        } else {
            x += marginOfError;
        }


        if (x > 0){
            return -((1 - Math.cos(x * Math.PI / smoothness)) / 2) * maxTurning;
        } else {
            return ((1 - Math.cos(x * Math.PI / smoothness)) / 2) * maxTurning;
        }
    }

    static BNO055IMU imu;
    public static double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) {
            angle += Math.PI * 2;
        }
        angle = Math.PI * 2 - angle;

        return angle;
    }
    static void setPower(double p){
        frontRight.setPower(-p);
        frontLeft .setPower(p);
        backRight .setPower(-p);
        backLeft  .setPower(p);
    }
}
