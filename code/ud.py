#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# 串口配置
serial_port = '/dev/ttyUSB0'  # Arduino串口，根据实际情况修改
baud_rate = 9600              # 与Arduino通信的波特率
def move_callback(vel_info):
# message_to_send=raw_input("Enter message to send to Arduino: ")+'\n'  # 定义要发送的消息
    vel=str(int(abs(vel_info.linear.z*45)))
    if vel_info.linear.z>0:
        message_to_send='$'+'U'+vel+'\n'
    else:
        message_to_send='$'+'D'+vel+'\n'
    write_to_port(ser, message_to_send.encode())  # 发送消息给Arduino

    # # 等待Arduino的回复
    # while not rospy.is_shutdown() and not ser.in_waiting:
    #     rospy.sleep(0.1)  # 短暂等待，避免忙等

    # # 读取Arduino的回复
    # reply = read_from_port(ser)
    # rospy.loginfo("Received reply from Arduino: %s", reply)




def write_to_port(ser, message):
    rospy.loginfo("Sending to Arduino: %s", message)
    ser.write(message.encode('utf-8'))  # 发送字符串到Arduino

def read_from_port(ser):
    while ser.in_waiting:
        reading = ser.read().decode('utf-8')  # 读取一个字符
        rospy.loginfo("Data from Arduino: %s", reading)
        return reading  # 返回读取的数据





if __name__ == "__main__":
    rospy.init_node('serial_node', anonymous=True)
    # 初始化串口
    try:
        ser = serial.Serial(serial_port, baud_rate)
    except serial.SerialException as e:
        # 如果串口初始化失败，抛出自定义异常
        raise RuntimeError("无法初始化串口 {}: {}".format(serial_port, e))
    except Exception as e:
        # 捕获其他可能的异常
        raise RuntimeError("意外的错误: {}".format(e))

    move_sub=rospy.Subscriber('/info', Twist, move_callback)
    rospy.spin()
