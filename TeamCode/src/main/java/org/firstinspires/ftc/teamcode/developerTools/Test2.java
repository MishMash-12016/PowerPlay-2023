package org.firstinspires.ftc.teamcode.developerTools;

import android.telephony.AccessNetworkConstants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.grabber;

@TeleOp
public class Test2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("b", gamepad1.b);
            telemetry.update();
        }
    }
}
