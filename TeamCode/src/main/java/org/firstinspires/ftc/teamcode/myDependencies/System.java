package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class System {
    // region SYSTEM CONSTANTS
    public enum DriveMode {}
    public enum ScoreState {}
    public enum CollectionState {}

    public static class LedColor {}

    public static final InterruptedException stopRequest = new InterruptedException("stop is requested");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion SYSTEM CONSTANTS

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static int currentConeHeight;
    // endregion SYSTEM VARIABLES

    // region INITIALIZATION
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        System.hardwareMap = hardwareMap;
        System.telemetry   = telemetry  ;
        System.gamepad1    = gamepad1   ;
        System.gamepad2    = gamepad2   ;

        isStopRequested = false;
        currentConeHeight = 0;

        arm.initialize();
        driveTrain.initialize();
        elevator.initialize();
        grabber.initialize();
        puffer.initialize();

    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static final Thread elevatorController = new Thread();
    public static final Thread driveController = new Thread();

    public static void terminate(){

    }
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

    // region DRIVER CONTROLLED FUNCTIONS
    static class manual{
        public static final Thread score = new Thread();
        public static final Thread collect = new Thread();
    }
    // endregion DRIVER CONTROLLED FUNCTIONS

    // region AUTONOMOUS FUNCTIONS
    static class auto{
        public static final Thread autoCycle = new Thread();

        private void score(boolean prepareForNext, int nextConeNum){}
        private void collect(int coneNum){}
    }
    // endregion AUTONOMOUS FUNCTIONS
}
