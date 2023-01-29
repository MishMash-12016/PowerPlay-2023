package org.firstinspires.ftc.teamcode.developerTools;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        RevBlinkinLedDriver light = RobotSystem.hardwareMap.get(RevBlinkinLedDriver.class, "led");

        while (opModeIsActive()){
            if (gamepad1.a){
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if (gamepad1.b){
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            else if (gamepad1.x){
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            else if (gamepad1.y){
                light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
        }
    }
}
