/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "遥控测试v4.0.3(Encoder)", group = "Iterative OpMode")
public class FieldCentricOpMode extends OpMode {
    // 记录运行时间的计时器
    private final ElapsedTime runtime = new ElapsedTime();
    //底部夹子的两个舵机
    //down_clip_head控制夹子的旋转
    //down_clip_hand控制夹子的抓放
    private Servo down_clip_head;
    private Servo down_clip_hand;
    //舵机夹子,顶部的那个,目前还没装好
    private Servo clip;
    private Servo top_clip_hand;
    private Servo top_clip_head;
    // 四个底盘电机
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;

    //抬升电机
    private DcMotor lift;
    // 惯性测量单元
    private IMU imu;
    private double multiplier;

    @Override
    public void init() {
        multiplier=1.1;
        // 向遥测发送初始化开始信息
        telemetry.addData("初始化" ,"启动");
        //初始化电机
        {
            down_clip_hand=hardwareMap.get(Servo.class,"DownClipHand");
//            down_clip_hand.setPosition(0.2);
//            down_clip_hand.scaleRange(0.3,0.7);//限制范围，待测试
            down_clip_head=hardwareMap.get(Servo.class,"DownClipHead");
            top_clip_hand=hardwareMap.get(Servo.class,"TopClipHand");
            top_clip_head=hardwareMap.get(Servo.class,"TopClipHead");
//            down_clip_head.setPosition(0);
//            down_clip_hand.setDirection(Servo.Direction.REVERSE);
            //舵机夹子
            clip = hardwareMap.get(Servo.class,"Clip");
            clip.setPosition(0);
//            top_clip_hand.setPosition(0);

            //Servo.Direction.REVERSE
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
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 向遥测发送初始化完成信息
        telemetry.addData("初始化", "完毕");
        telemetry.addData("编码器初始",lift.getCurrentPosition());
    }
    @Override
    public void init_loop() {
        // 初始化循环，这里为空，可能是后续需要添加的功能预留
    }
    @Override
    public void start() {
        // 开始时重置运行时间
        runtime.reset();
    }
    private void FieldCentricMecanum(){
        // 获取游戏手柄左摇杆的 y 轴和 x 轴的值
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        // 获取游戏手柄右摇杆的 x 轴的值并取反
        double rx = -gamepad1.right_stick_x;
        // 如果按下游戏手柄的选项按钮，则重置 IMU 的偏航角
        double lt=gamepad2.left_trigger;
        double rt=gamepad2.right_trigger;
        // Reset the motor encoder so that it reads zero ticks

        lift.setPower((rt>0.1?rt-0.1:0)-(lt>0.1?lt-0.1:0));
        if(gamepad2.left_stick_x>0.1) {
            down_clip_head.setPosition(Math.min(1,Math.max(0,down_clip_head.getPosition() + gamepad2.left_stick_x / 100)));
        }
        if(gamepad2.left_stick_x<-0.1) {
            down_clip_head.setPosition(Math.min(1,Math.max(0,down_clip_head.getPosition()+gamepad2.left_stick_x / 100)));
        }
        if(gamepad2.left_stick_y>0.1) {
            down_clip_hand.setPosition(Math.min(0.75,Math.max(0.3,down_clip_hand.getPosition() + gamepad2.left_stick_y / 200)));
        }
        if(gamepad2.left_stick_y<-0.1) {
            down_clip_hand.setPosition(Math.min(0.75,Math.max(0.3,down_clip_hand.getPosition() + gamepad2.left_stick_y / 200)));
        }

        if(gamepad2.right_stick_x>0.1) {
            top_clip_head.setPosition(Math.min(1,Math.max(0,top_clip_head.getPosition() + gamepad2.right_stick_x / 100)));
        }
        if(gamepad2.right_stick_x<-0.1) {
            top_clip_head.setPosition(Math.min(1,Math.max(0,top_clip_head.getPosition()+gamepad2.right_stick_x / 100)));
        }
        if(gamepad2.right_stick_y>0.1) {
            top_clip_hand.setPosition(Math.min(0.75, Math.max(0.3, top_clip_hand.getPosition() + gamepad2.right_stick_y / 200)));
        }
        if(gamepad2.right_stick_y<-0.1) {
            top_clip_hand.setPosition(Math.min(0.75,Math.max(0.3,top_clip_hand.getPosition() + gamepad2.right_stick_y / 200)));
        }

        if(gamepad2.left_bumper) {
//            clip.setPosition(0);//0degrees
            clip.setPosition(down_clip_hand.getPosition()+0.001);
        }
        if(gamepad2.right_bumper) {
//            clip.setPosition(0.5);//90degrees
            clip.setPosition(down_clip_hand.getPosition()-0.001);
        }
        if (gamepad1.y) {
            telemetry.addData("按键","[y]");
//            lift.setPower(0.3);
//            if(multiplier<2.5)multiplier+=0.001;
//            imu.resetYaw();
        }
        if (gamepad1.x) {
            telemetry.addData("按键", "[x]");
//            lift.setPower(-0.3);
//            if (multiplier>0.1) multiplier -= 0.001;
//            imu.resetYaw();
        }

        if(gamepad1.b) {
            lift.setPower(0);
            telemetry.addData("按键","[B]");
        }
        if (gamepad1.options) {
            telemetry.addData("按键","[START]");
            imu.resetYaw();
        }
        // 获取机器人的偏航角（以弧度为单位）
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // 将运动方向相对于机器人的旋转进行旋转
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        // 抵消不完美的平移
        rotX = rotX * multiplier;
        //分母是最大的电机功率（绝对值）或1，这确保所有功率保持相同的比例，但仅当至少一个超出范围[-1,1]时。
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        // 计算四个电机的功率
        double front_left_power = (rotY + rotX + rx) / denominator;
        double rear_left_power = (rotY - rotX + rx) / denominator;
        double front_right_power = (rotY - rotX - rx) / denominator;
        double rear_right_power = (rotY + rotX - rx) / denominator;
        // 设置四个电机的功率
        front_left.setPower(front_left_power);
        rear_left.setPower(rear_left_power);
        front_right.setPower(front_right_power);
        rear_right.setPower(rear_right_power);
        telemetry.addData("舵机","[Clip:%.2f] [DCHead:%.2f] [DCHand:%.2f] [TCHead:%.2f] [TCHand:%.2f]",clip.getPosition(),down_clip_head.getPosition(),down_clip_hand.getPosition(),top_clip_head.getPosition(),top_clip_hand.getPosition());
//        telemetry.addData("抬升按钮","[LT:%.2f],[RT:%.2f]",lt,rt);
        telemetry.addData("抬升功率","[%.2f]",(rt>0.1?rt-0.1:0)-(lt>0.1?lt-0.1:0));
        telemetry.addData("编码器",lift.getCurrentPosition());
//        telemetry.addData("偏移倍数","%.2f",multiplier);
        // 向遥测发送机器人的角度信息
        telemetry.addData("角度", "(%.2f)", botHeading);
        // 向遥测发送四个电机的功率信息
        telemetry.addData("电机功率", "左前:(%.2f) 右前:(%.2f) 左后:(%.2f) 右后:(%.2f)", front_left_power, front_right_power, rear_left_power, rear_right_power);
    }
    @Override
    public void loop() {
        // 调用场心麦卡纳姆驱动方法
        FieldCentricMecanum();
        // 向遥测发送运行时间信息
        telemetry.addData("运行时间", runtime);
    }
    @Override
    public void stop() {
        // 停止时的操作，这里为空，可能是后续需要添加的功能预留
    }
}