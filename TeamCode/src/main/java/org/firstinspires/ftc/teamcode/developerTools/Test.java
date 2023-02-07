package org.firstinspires.ftc.teamcode.developerTools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.elevator;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class Test extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        elevator.controller.start();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("position", elevator.getCurrentPosition());
            telemetry.update();
            if (gamepad1.a){
                elevator.wantedPosition = 13000;
            }
            if (gamepad1.b){
                elevator.wantedPosition = 0;
            }
        }
    }
}
