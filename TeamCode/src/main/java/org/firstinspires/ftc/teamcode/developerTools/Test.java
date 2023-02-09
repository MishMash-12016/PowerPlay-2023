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
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.util.BNO055IMUUtil;


//@Deprecated
@Autonomous(group = "developer tools")
public class Test extends LinearOpMode {
    static BNO055IMU imu;

    static DcMotor frontRight;
    static DcMotor frontLeft ;
    static DcMotor backRight ;
    static DcMotor backLeft  ;
    @Override
    public void runOpMode(){
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft  = hardwareMap.dcMotor.get("frontLeft" );
        backRight  = hardwareMap.dcMotor.get("backRight" );
        backLeft   = hardwareMap.dcMotor.get("backLeft"  );

        // flipping the two left driving motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft .setDirection(DcMotorSimple.Direction.REVERSE);

        // region INITIALIZE IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        // endregion

        waitForStart();

        double wantedAngle = Math.toRadians(-160);
        wantedAngle *= -1;
        wantedAngle -= Math.toRadians(90);

        while (opModeIsActive() && (wantedAngle - getAngle() + Math.PI * 10) % (Math.PI * 2) > 0.05){
            telemetry.addData("angle", getAngle());
            telemetry.update();

            if ((wantedAngle - getAngle() + Math.PI * 10) % (Math.PI * 2) > -Math.PI && (wantedAngle - getAngle() + Math.PI * 10) % (Math.PI * 2) < Math.PI)
                turn(-0.3);
            else
                turn(0.3);
        }
        turn(0);

    }
    public static double getAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) {
            angle += Math.PI * 2;
        }
        angle = Math.PI * 2 - angle;

        return angle + driveTrain.startingAngle;
    }

    public static void turn(double p){
        frontRight.setPower(p);
        frontLeft .setPower(-p);
        backRight .setPower(p);
        backLeft  .setPower(-p);
    }
}
