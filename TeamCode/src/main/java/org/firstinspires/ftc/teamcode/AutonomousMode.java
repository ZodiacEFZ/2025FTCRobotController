package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="Robot: Auto Drive & Move", group="Robot")
public class AutonomousMode extends OpMode {
    private class Zodiac {
        LynxModule controlHub;
        private Servo down_clip_head;//down_clip_head控制夹子的旋转
        private Servo down_clip_hand;//down_clip_hand控制夹子的抓放

        private Servo clip;//舵机夹子,顶部的那个,目前还没装好

        private DcMotor front_left;// 四个底盘电机
        private DcMotor front_right;
        private DcMotor rear_left;
        private DcMotor rear_right;

        private DcMotor lift;//抬升电机

        private IMU imu;// 惯性测量单元
        private double multiplier=1.1;

        public void init(){
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            {
                down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
//            down_clip_hand.setPosition(0.2);
//            down_clip_hand.scaleRange();//限制范围，待测试
                down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
                down_clip_head.setPosition(0);
                //舵机夹子
                clip = hardwareMap.get(Servo.class,"Clip");
                clip.setPosition(0);
                //抬升电机
                lift = hardwareMap.get(DcMotor.class,"Lift");
                // 从硬件映射中获取四个底盘电机
                front_left = hardwareMap.get(DcMotor.class, "frontLeft");
                front_right = hardwareMap.get(DcMotor.class, "frontRight");
                rear_left = hardwareMap.get(DcMotor.class, "rearLeft");
                rear_right = hardwareMap.get(DcMotor.class, "rearRight");
                // 设置电机的转动方向
                lift.setDirection(DcMotorSimple.Direction.REVERSE);
                front_left.setDirection(DcMotorSimple.Direction.FORWARD);
                front_right.setDirection(DcMotorSimple.Direction.REVERSE);
                rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
                rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
                // 设置电机在功率为零时的行为为制动
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            {
                // 从硬件映射中获取 IMU
                imu = hardwareMap.get(IMU.class, "imu");
                // 调整方向参数以匹配机器人
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                // 如果没有这个，REV Hub 的方向将被假定为标志朝上 / USB 朝前
                imu.initialize(parameters);
                // 重置 IMU 的偏航角
                imu.resetYaw();
            }
        }

        private class PID_control{
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
                controlHub.clearBulkCache();
                double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                TurnCounterclockwise(pid.calc((currentDegree-origDegree), degree));
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
                controlHub.clearBulkCache();
                double currentDegree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                TurnClockwise(pid.calc((origDegree-currentDegree), degree));
            }
            stopChassis();
        }
        private void TurnCounterclockwise(double power){
            if((power < 0.07) && (power >= 0.0)){
                power = 0.0;
            }
            setChassisPower(power, -power, power, -power);
        }
        private void TurnClockwise(double power){
            if((power < 0.07) && (power >= 0.0)){
                power = 0.0;
            }
            setChassisPower(-power, power, -power, power);
        }
        public void forward(double seconds, double CPower) {
            ElapsedTime timer = new ElapsedTime();
            telemetry.addData("Move:","(F)Forward...");
            telemetry.update();
            setChassisPower(CPower, CPower, CPower, CPower);
            while (timer.seconds() <= seconds) {}
            stopChassis();
        }
        public void backward(double seconds, double CPower) {
            ElapsedTime timer = new ElapsedTime();
            telemetry.addData("Move:","(B)Backward...");
            telemetry.update();
            setChassisPower(-CPower, -CPower, -CPower, -CPower);
            while (timer.seconds() <= seconds) {}
            stopChassis();
        }
        public void leftward(double seconds, double CPower) {
            ElapsedTime timer = new ElapsedTime();
            telemetry.addData("Move:","(L)Leftward...");
            telemetry.update();
            setChassisPower(-CPower, CPower, CPower, -CPower);
            while (timer.seconds() <= seconds) {}
            stopChassis();
        }
        public void rightward(double seconds, double CPower) {
            ElapsedTime timer = new ElapsedTime();
            telemetry.addData("Move:","(R)Rightward...");
            telemetry.update();
            setChassisPower(CPower, -CPower, -CPower, CPower);
            while (timer.seconds() <= seconds) {}
            stopChassis();
        }
        public void stopChassis() {
            setChassisPower(0.0, 0.0, 0.0, 0.0);
            telemetry.addData("Move:","(S)Stop");
            telemetry.update();
        }
        public void setChassisPower(double front_left_power, double front_right_power, double rear_left_power, double rear_right_power) {
            front_left.setPower(front_left_power);
            front_right.setPower(front_right_power);
            rear_left.setPower(rear_left_power);
            rear_right.setPower(rear_right_power);
        }
    }

    private final ElapsedTime runtime = new ElapsedTime();

    private Zodiac robot;

    private final double DefaultCPower = 0.5;

    private int steps = 0; /*初始 0
                            前进 1
                            左转 2
                            */

    @Override
    public void init() {
        robot = new Zodiac();
        telemetry.addData("初始化" ,"启动");
        robot.init();
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
        LoopCode();
        telemetry.addData("运行时间", runtime);
        telemetry.addData("角度", "(%.2f)", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    private void LoopCode(){
        switch (steps){
            case (0): {
                telemetry.addData("Steps:", String.format("%d (初始)", steps));
                steps++;
                break;
            }
            case (1): {
                telemetry.addData("Steps:", String.format("%d (前进)", steps));
                robot.forward(0.5, DefaultCPower);
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() <= 0.5) {}
                steps++;
                break;
            }
            case (2): {
                telemetry.addData("Steps:", String.format("%d (左转)", steps));
                robot.leftTurn(90, 5);
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() <= 0.5) {}
                steps++;
                break;
            }
            default:{
                telemetry.addData("Steps:", String.format("%d (停止🛑)", steps));
                robot.stopChassis();
            }
        }
    }



}
