package org.firstinspires.ftc.teamcode.developerTools;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.elevator;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test extends LinearOpMode {
    static Thread x;
    @Override
    public void runOpMode() {
        x = new Thread(() -> {
            while (!Test.x.isInterrupted()) {
                telemetry.addData("running", true);
                telemetry.update();
            }
            telemetry.addData("running", false);
            telemetry.update();
        });

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        x.start();
        while (opModeIsActive()) {
           if (gamepad1.b){
               Test.x.interrupt();
           }
        }
    }
}
