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
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.elevator;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@TeleOp
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() {
        RobotSystem.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        if (isStopRequested()) {
            RobotSystem.terminate();
            return;
        }
        resetRuntime();

        driveTrain.angleHolder.start();

        while (opModeIsActive()) {
            if      (gamepad1.dpad_up   ) driveTrain.wantedAngle = Math.PI * 0  ;
            else if (gamepad1.dpad_right) driveTrain.wantedAngle = Math.PI * 0.5;
            else if (gamepad1.dpad_down ) driveTrain.wantedAngle = Math.PI * 1  ;
            else if (gamepad1.dpad_left ) driveTrain.wantedAngle = Math.PI * 1.5;
            telemetry.addData("position", driveTrain.getRobotAngle());
            telemetry.addData("wantedPosition", driveTrain.wantedAngle);
            telemetry.update();
        }

        RobotSystem.terminate();
    }
}
