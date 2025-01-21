package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="03 只挂高杆，不倒退，不向右平移", group="Autonomous")
public class Autonomous3 extends OpMode {
    //LynxModule controlHub;
    private Servo down_clip_head;//down_clip_head控制夹子的旋转
    private Servo down_clip_hand;//down_clip_hand控制夹子的抓放
    private ServoImplEx down_clip_arm;//舵机夹子,顶部的那个,目前还没装好
    //private Servo top_clip_head;
    private Servo top_clip_hand;
    private Servo top_clip_arm;
    private Servo head;
    private DcMotor front_left;// 四个底盘电机
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private DcMotor lift;//抬升电机
    private DcMotor intake;
    private IMU imu;// 惯性测量单元
    //private boolean DCstate = true, LB_last_pressed = false; // false for open; true for close
    //private boolean TCstate = true, RB_last_pressed = false; // false for open; true for close


    private final ElapsedTime runtime = new ElapsedTime();

    private Values values = new Values();

    public void init_robot(){
        //controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        //controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        telemetry.setAutoClear(false);
        //Initialize motors
        {
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            down_clip_arm=(ServoImplEx) hardwareMap.get(Servo.class,"DownClipArm");
            down_clip_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));

            top_clip_arm=hardwareMap.get(Servo.class,"TopClipArm");
            //top_clip_head=hardwareMap.get(Servo.class,"TopClipHead");
            top_clip_hand = hardwareMap.get(Servo.class,"TopClipHand");

            head = hardwareMap.get(Servo.class,"Head");

            down_clip_hand.setPosition(values.clipPositions.get("DC_close"));
            top_clip_hand.setPosition(values.clipPositions.get("TC_close"));
            down_clip_arm.setPosition(values.armPositions.get("DC_arm_init"));
            down_clip_head.setPosition(values.armPositions.get("DC_head_init"));
            head.setPosition(values.headPositions.get("Head_up"));

            intake = hardwareMap.get(DcMotor.class,"Intake");
            //抬升电机
            lift = hardwareMap.get(DcMotor.class,"Lift");
            // 从硬件映射中获取四个底盘电机
            front_left = hardwareMap.get(DcMotor.class, "frontLeft");
            front_right = hardwareMap.get(DcMotor.class, "frontRight");
            rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
            rear_right = hardwareMap.get(DcMotor.class, "rearRight");
            // 设置电机的转动方向
            intake.setDirection(DcMotor.Direction.FORWARD);
            lift.setDirection(DcMotor.Direction.REVERSE);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.REVERSE);
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            rear_right.setDirection(DcMotor.Direction.REVERSE);
            // 设置电机在功率为零时的行为为制动
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setTargetPosition(0);
        }
        //Initialize IMU & reset yaw
        {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // 如果没有这个，REV Hub 的方向将被假定为标志朝上 / USB 朝前
            imu.initialize(parameters);
            imu.resetYaw();
        }
    }

    /*public void telemetryAddData(String caption, Object value){
        telemetry.addData(caption, value);
    }
    public void telemetryUpdate(){
        telemetry.update();
    }*/

    /*private class PID_control{
        double k_p, k_i, k_d, maxi, maxoutput;
        ElapsedTime start_timer;
        double previous_time = 0.0, previous_error = 0.0, output;
        double i = 0.0;

        public PID_control(double kp, double ki, double kd, double max_i, double max_output){
            k_p = kp;
            k_i = ki;
            k_d = kd;
            maxi = max_i;
            maxoutput = max_output;
        }

        public void initTimer(){
            start_timer = new ElapsedTime();
            previous_time = 0.0;
            previous_error = 0.0;
            output = 0.0;
            i = 0.0;
        }

        public double calc(double current_position, double desire_position){
            double current_time = start_timer.seconds();
            double current_error = desire_position-current_position;

            double p = k_p * current_error;
            i += k_i * (current_error * (current_time - previous_time));
            if(i > maxi){
                i = maxi;
            }
            else if(i < -maxi){
                i = -maxi;
            }
            double d = k_d * (current_error - previous_error) / (current_time - previous_time);
            previous_error = current_error;
            previous_time = current_time;
            output = p + i + d;
            if(output > maxoutput){
                output = maxoutput;
            }
            else if(output < -maxoutput){
                output = -maxoutput;
            }
            return output;
        }
    }
    double turn_k_p = 0.001, turn_k_i = 0.01, turn_k_d = 0.005, turn_maxi = 0.5, turn_maxoutput = 1.0;

    public void leftTurn(double degree, double seconds){
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","（←)Turning Left...");
        telemetry.update();
        double origDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID_control pid = new PID_control(turn_k_p, turn_k_i, turn_k_d, turn_maxi, turn_maxoutput);
        pid.initTimer();
        while (timer.seconds() <= seconds) {
            //controlHub.clearBulkCache();  //No need
            double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //TurnCounterclockwise(pid.calc((currentDegree-origDegree), degree));
            TurnCounterclockwise(pid.calc(currentDegree, (origDegree + degree)));
        }
        stopChassis();
    }
    public void rightTurn(double degree, double seconds){
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Move:","（→)Turning Right...");
        telemetry.update();
        double origDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID_control pid = new PID_control(turn_k_p, turn_k_i, turn_k_d, turn_maxi, turn_maxoutput);
        pid.initTimer();
        while (timer.seconds() <= seconds) {
            //controlHub.clearBulkCache();  //No need
            double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            TurnClockwise(pid.calc((origDegree-currentDegree), degree));
        }
        stopChassis();
    }
    private void TurnCounterclockwise(double power){
            //if((power < 0.07) && (power >= 0.0)){
            //    power = 0.0;
            //}
        setChassisPower(power, -power, power, -power);
    }
    private void TurnClockwise(double power){
            //if((power < 0.07) && (power >= 0.0)){
            //    power = 0.0;
            //}
        setChassisPower(-power, power, -power, power);
    }
    */

    private enum AutoState {
        START0,
        FORWARD_LIFT1,
        PUT2,
        RETRACT_BACKWARD3,
        //RIGHTWARD4,
        //TURN5,
        END6
    };
    private AutoState autoState = AutoState.START0;

    /*private enum ChassisState {
        STOP,
        FORWARD,
        BACKWARD,
        LEFTWARD,
        RIGHTWARD
    };*/
    private enum ChassisState {
        STOP,
        MOVE
    };
    private ChassisState chassisState = ChassisState.STOP;
    private double chassisStartTime = 0.0, chassisSetTime = 0.0, /*chassisPower = 0.0,*/ chassisX = 0.0, chassisY = 0.0, real_chassisRX = 0.0;

    private double StartStatePutTime = 0.0;

    //loopMoveChassis() needes to be executed every loop
    private void loopMoveChassis(){
        double currentTime = runtime.seconds();
               
        if(chassisState == ChassisState.STOP){
            chassisX = 0.0;
            chassisY = 0.0;
            real_chassisRX = 0.0;
            setChassisPower(0.0, 0.0, 0.0, 0.0);;
            chassisStartTime = 0.0;
            chassisSetTime = 0.0;
            //chassisPower = 0.0;
            telemetry.addData("Move:", "🛑Stop");
        }
        else {
            if((currentTime-chassisStartTime) >= chassisSetTime){
                setChassisPower(0.0, 0.0, 0.0, 0.0);;
                chassisState = ChassisState.STOP;
            }
            else {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // 将运动方向相对于机器人的旋转进行旋转
                double rotX = chassisX * Math.cos(botHeading) - chassisY * Math.sin(botHeading);
                double rotY = chassisX * Math.sin(botHeading) + chassisY * Math.cos(botHeading);
                // 抵消不完美的平移
                rotX = rotX * values.chassisMultiplier;
                //分母是最大的电机功率（绝对值）或1，这确保所有功率保持相同的比例，但仅当至少一个超出范围[-1,1]时。
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(real_chassisRX), 1);
                // 计算四个电机的功率
                double front_left_power = (rotY + rotX + real_chassisRX) / denominator;
                double rear_left_power = (rotY - rotX + real_chassisRX) / denominator;
                double front_right_power = (rotY - rotX - real_chassisRX) / denominator;
                double rear_right_power = (rotY + rotX - real_chassisRX) / denominator;
                // 设置四个电机的功率
                front_left.setPower(front_left_power);
                rear_left.setPower(rear_left_power);
                front_right.setPower(front_right_power);
                rear_right.setPower(rear_right_power);

                telemetry.addData("Move:", "Moving...");
            }
        }
        
        /*switch (chassisState) {
            case STOP: {
                setChassisPower(0.0, 0.0, 0.0, 0.0);;
                chassisStartTime = 0.0;
                chassisSetTime = 0.0;
                chassisPower = 0.0;
                telemetry.addData("Move:", "🛑Stop");
                break;
            }
            case FORWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    //setChassisPower(chassisPower, chassisPower, chassisPower, chassisPower);
                    telemetry.addData("Move:","(↑)Forward...");
                }
                break;
            }
            case BACKWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    //setChassisPower(-chassisPower, -chassisPower, -chassisPower, -chassisPower);
                    telemetry.addData("Move:","(↓)Backward...");
                }
                break;
            }
            case LEFTWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    setChassisPower(-chassisPower, chassisPower, chassisPower, -chassisPower);
                    telemetry.addData("Move:","(←)Leftward...");
                }
                break;
            }
            case RIGHTWARD: {
                if((currentTime-chassisStartTime) >= chassisSetTime){
                    setChassisPower(0.0, 0.0, 0.0, 0.0);;
                    chassisState = ChassisState.STOP;
                }
                else {
                    setChassisPower(chassisPower, -chassisPower, -chassisPower, chassisPower);
                    telemetry.addData("Move:","(→)Rightward...");
                }
                break;
            }
            default: {
                setChassisPower(0.0, 0.0, 0.0, 0.0);;
                telemetry.addData("Chassis:", "🛑Stop(Default)");
                break;
            }
        }*/
    }
    
    private void forwardState(double seconds, double Cpower){
        chassisState = ChassisState.MOVE;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisY = -Cpower;
        chassisX = 0.0;
        real_chassisRX = 0.0;
    }
    private void backwardState(double seconds, double Cpower){
        chassisState = ChassisState.MOVE;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisY = Cpower;
        chassisX = 0.0;
        real_chassisRX = 0.0;
    }
    private void leftwardState(double seconds, double Cpower){
        chassisState = ChassisState.MOVE;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisY = 0.0;
        chassisX = -Cpower;
        real_chassisRX = 0.0;
    }
    private void rightwardState(double seconds, double Cpower){
        chassisState = ChassisState.MOVE;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisY = 0.0;
        chassisX = Cpower;
        real_chassisRX = 0.0;
    }
    private void turnState(double seconds, double gp_rx){
        chassisState = ChassisState.MOVE;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisY = 0.0;
        chassisX = 0.0;
        real_chassisRX = -gp_rx;
    }

    /*private void forwardState(double seconds, double CPower) {
        chassisState = ChassisState.FORWARD;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
        setChassisPower(-chassisPower, -chassisPower, -chassisPower, -chassisPower);
    }
    private void backwardState(double seconds, double CPower) {
        chassisState = ChassisState.BACKWARD;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
        setChassisPower(chassisPower, chassisPower, chassisPower, chassisPower);
    }
    private void leftwardState(double seconds, double CPower) {
        chassisState = ChassisState.LEFTWARD;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
        setChassisPower(-chassisPower, chassisPower, chassisPower, -chassisPower);
    }
    private void rightwardState(double seconds, double CPower) {
        chassisState = ChassisState.RIGHTWARD;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        chassisPower = CPower;
        setChassisPower(chassisPower, -chassisPower, -chassisPower, chassisPower);
    }
    private void rightwardStateMod(double seconds, double CPower) {
        chassisState = ChassisState.RIGHTWARD;
        chassisStartTime = runtime.seconds();
        chassisSetTime = seconds;
        //chassisPower = CPower;
        //setChassisPower(chassisPower, -chassisPower, -chassisPower, chassisPower);
        setChassisPower(0.5, -0.7, -0.5, 0.5);
    }*/
    private void stopState() {
        chassisState = ChassisState.STOP;
    }
    private void setChassisPower(double front_left_power, double front_right_power, double rear_left_power, double rear_right_power) {
        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        rear_left.setPower(rear_left_power);
        rear_right.setPower(rear_right_power);
    }
    /*private void stopChassis() {
        setChassisPower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData("Move:","(S)Stop");
        telemetry.update();
    }*/

    @Override
    public void init() {
        telemetry.addData("初始化" ,"启动");
        init_robot();
        telemetry.addData("初始化", "完毕");
    }

    @Override
    public void start() {
        runtime.reset();// 开始时重置运行时间
    }
    @Override
    public void init_loop() {
        // 初始化循环，这里为空，可能是后续需要添加的功能预留
    }
    @Override
    public void loop() {
        loopMoveChassis();

        int liftCurrentPosition = lift.getCurrentPosition();
        switch (autoState){
            case START0: {
                telemetry.addData("State0:", "0 START");
                telemetry.update();
                // Set positions for next state

                head.setPosition(values.headPositions.get("Head_down"));
                // Set forward
                forwardState(6, 0.4*values.AutonomousCPower);
                // Set lift
                top_clip_arm.setPosition(values.armPositions.get("TC_arm_up"));//add
                lift.setTargetPosition(values.liftPositions.get("up"));
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                // Change state
                StartStatePutTime = runtime.seconds();
                autoState = AutoState.FORWARD_LIFT1;
                telemetry.addLine("==================");
                telemetry.update();
                break;
            }
            case FORWARD_LIFT1: {
                telemetry.addData("State1:", "1 FORWARD & LIFT");
                telemetry.update();
                double lift_current_A = ((DcMotorEx)lift).getCurrent(CurrentUnit.AMPS);
                if(lift_current_A >= 3.0){
                    lift.setPower(0.5);
                }
                //wait for forward & lift
                if ((Math.abs(liftCurrentPosition - values.liftPositions.get("up")) < 50) && chassisState== ChassisState.STOP && (runtime.seconds()-StartStatePutTime)>=6) {
                    // Set positions for next state
                    lift.setTargetPosition(values.liftPositions.get("put"));
                    StartStatePutTime = runtime.seconds();
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);
                    //change state
                    autoState = AutoState.PUT2;
                    telemetry.addLine("==================");
                    telemetry.update();
                }
                break;
            }
            case PUT2: {
                telemetry.addData("State2:","2 PUT");
                telemetry.update();
                double lift_current_A = ((DcMotorEx)lift).getCurrent(CurrentUnit.AMPS);
                if(lift_current_A >= 3.0){
                    lift.setPower(0.5);
                }
                if(Math.abs(liftCurrentPosition - values.liftPositions.get("put")) < 50 && (runtime.seconds()-StartStatePutTime)>=5) {
                    top_clip_hand.setPosition(values.clipPositions.get("TC_open"));

                    lift.setTargetPosition(values.liftPositions.get("zero"));
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(1);

                    //backwardState(5,0.5*values.AutonomousCPower);

                    top_clip_arm.setPosition(values.armPositions.get("TC_arm_down"));

                    autoState = AutoState.RETRACT_BACKWARD3;
                    StartStatePutTime = runtime.seconds();
                    telemetry.addLine("==================");
                    telemetry.update();
                }
                break;
            }
            case RETRACT_BACKWARD3:{
                telemetry.addData("State3:","3 RETRACT_BACKWARD");
                telemetry.update();

                if(Math.abs(liftCurrentPosition - values.liftPositions.get("zero")) < 50 && (runtime.seconds()-StartStatePutTime)>=5) {
                    //rightwardState(7, 0.7*values.AutonomousCPower);
                    autoState = AutoState.END6;
                    telemetry.addLine("==================");
                    telemetry.update();
                }

                break;
            }
            /*case RIGHTWARD4: {
                telemetry.addData("State4:", "4 RIGHTWARD");
                telemetry.update();
                if(chassisState== ChassisState.STOP) {
                    //turnState(0.3, 0.5);
                    //autoState = AutoState.TURN5;
                    autoState = AutoState.END6;
                    telemetry.addLine("==================");
                    telemetry.update();
                }

                //stopState();
                break;
            }*/
            /*case TURN5: {
                telemetry.addData("State5:", "5 TURN");
                telemetry.update();
                if(chassisState==ChassisState.STOP) {
                    autoState = AutoState.END6;
                    telemetry.addLine("==================");
                    telemetry.update();
                }

                //stopState();
                break;
            }*/
            case END6:{
                telemetry.addData("State6:","6 END");
                telemetry.update();
                stopState();
                break;
            }
            default:{
                telemetry.addData("State00:", "WAIT(default)");
                telemetry.update();
                stopState();
            }
        }

        telemetry.addData("运行时间", runtime);
        telemetry.addData("角度(Degrees)", "(%.2f)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("抬升位置", "(%d)", liftCurrentPosition);
        telemetry.update();
    }
    
}
