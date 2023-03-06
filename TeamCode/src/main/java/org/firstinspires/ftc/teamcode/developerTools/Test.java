package org.firstinspires.ftc.teamcode.developerTools;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Mat;


@Autonomous(group = "competition op modes")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.test.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.test.initializeTrajectories();

        waitForStart();

        try {
            RobotSystem.test.follow(RobotSystem.test.trajectories.get("startToTest"));
        } catch (Exception e) {
            RobotSystem.test.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

    }
}
