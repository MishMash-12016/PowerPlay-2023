package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.grabber;

@TeleOp(group = "competition op modes")
public class manualDrive extends LinearOpMode{
    @Override
    public void runOpMode() {
        RobotSystem.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        if (isStopRequested()) {
            RobotSystem.terminate();
            return;
        }
        resetRuntime();

        RobotSystem.startAllTeleOpControllers();

        while (opModeIsActive()){
            telemetry.addData("distance", grabber.distanceReading());
            telemetry.update();
        }

        RobotSystem.terminate();
    }
}