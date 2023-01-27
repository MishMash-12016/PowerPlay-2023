package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.System;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;

@TeleOp
public class manualDrive extends LinearOpMode{
    @Override
    public void runOpMode() {
        System.initialize(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        System.startAllControllers();
    }
}