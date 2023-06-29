#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import indydcp_client as client
import copy


def main():
    robot_ip = "192.168.0.8"    # 예시 STEP IP 주소
    robot_name = "NRMK-Indy10"   # IndyRP2의 경우 "NRMK-IndyRP2"
    indy = client.IndyDCPClient(robot_ip, robot_name) # indy 객체 생성

    indy.connect() # 로봇 연결
    vel  : int   = 9 # 1 ~ 9

    print(f'setting velocity: {vel:.2f}')
    # Setting 
    indy.set_joint_vel_level(vel)     # 1 ~ 9
    indy.set_task_vel_level(vel)     # 1 ~ 9

    indy.disconnect() # 연결 해제


if __name__ == '__main__':
    try:
        main()

    except Exception as e:
        print("[ERROR]", e)