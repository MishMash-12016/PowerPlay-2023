package org.firstinspires.ftc.teamcode.myDependencies;

import android.hardware.HardwareBuffer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.ElevatorPositions;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Gamepad;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Vector;

public class System {
    // region SYSTEM CONSTANTS
    public enum DrivingMode {}
    public enum ScoreState {}
    public enum CollectionState {}

    public static class LedColor {}

    public static final InterruptedException stopRequest = new InterruptedException("stop is requested");

    public static Gamepad gamepad1;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion SYSTEM CONSTANTS

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    // endregion SYSTEM VARIABLES

    // region INITIALIZATION
    public static void Initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        System.hardwareMap = hardwareMap;
        System.telemetry   = telemetry  ;
        System.gamepad1    = gamepad1   ;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public Thread elevatorController;
    public Thread driveController;
    public Thread gamepadController;
    // endregion SYSTEM FUNCTIONALITY

    // region GENERAL FUNCTIONALITY
    private static final ElapsedTime timePassed = new ElapsedTime();
    public void sleep(int wantedSleepTime) throws InterruptedException{
        timePassed.reset();
        while(wantedSleepTime < timePassed.milliseconds()){
            if (isStopRequested) {
                throw stopRequest;
            }
        }
    }
    // endregion GENERAL FUNCTIONALITY

    // region GAME PAD FUNCTIONALITY IMPORTER
    static class gamepad{
        public static boolean a = false;
        public static boolean b = false;
        public static boolean y = false;
        public static boolean x = false;

        public static boolean dpadUp    = false;
        public static boolean dpadLeft  = false;
        public static boolean dpadDown  = false;
        public static boolean dpadRight = false;

        public static boolean rightBumper = false;
        public static boolean leftBumper  = false;


        public static Vector2d leftJoystick  = new Vector2d(0, 0);
        public static Vector2d rightJoystick = new Vector2d(0, 0);


        public static double rightTrigger = 0;
        public static double leftTrigger  = 0;

    }
    // endregion GAME PAD FUNCTIONALITY IMPORTER

    // region DRIVER CONTROLLED FUNCTIONS
    static class manual{
        public Thread score;
        public Thread collect;
    }
    // endregion DRIVER CONTROLLED FUNCTIONS

    // region AUTONOMOUS FUNCTIONS
    static class auto{
        public Thread autoCycle;

        private void score(boolean prepareForNext, int nextConeNum){}
        private void collect(int coneNum){}
    }
    // endregion AUTONOMOUS FUNCTIONS
}
