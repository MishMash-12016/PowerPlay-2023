package org.firstinspires.ftc.teamcode.developerTools;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;

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


        while (opModeIsActive()){
        }
        RobotSystem.terminate();
    }
}
