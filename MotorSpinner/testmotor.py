# -*- coding: utf-8 -*-
import serial
from universalFuncs import *
from struct import *

class testmotor():
    def __init__(self, ser, id):
        self.ser = ser
        self.id = id
        
    def sendEnable(self):
        buff = [0,0,self.id,1,0,0,0,0,0,0,0,0,0,255,0]
        try:
            self.ser.write(bytes(buff))
        except:
            pass 

    def sendDisable(self):
        buff = [0,0,self.id,2,0,0,0,0,0,0,0,0,0,255,0]
        try:
            self.ser.write(bytes(buff))
        except:
            pass 

    def sendZero(self):
        buff = [0,0,self.id,3,0,0,0,0,0,0,0,0,0,255,0]
        try:
            self.ser.write(bytes(buff))
        except:
            pass 

    def sendCMD(self,p_des,dir,rot,v_des,t_ff):
        p_des = min(max(P_ANGLE_MIN, p_des), P_ANGLE_MAX)
        v_des = min(max(V_MIN, v_des), V_MAX)
        t_ff = min(max(T_MIN, t_ff), T_MAX)
        p_int = float_to_uint(p_des, P_ANGLE_MIN, P_ANGLE_MAX, 16)           
        v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
        t_int = float_to_uint(t_ff, T_FF_MIN, T_FF_MAX, 12)

        msg_byte1 = (0x01)<<4 | v_int>>8                              
        msg_byte2 = v_int&0xFF
        msg_byte3 = t_int>>4
        msg_byte4 = (t_int&0xF)<<4 | dir&0x01
        msg_byte5 = rot>>8
        msg_byte6 = rot&0xFF
        msg_byte7 = p_int>>8
        msg_byte8 = p_int&0xFF

        buff = [0,0,self.id,4,msg_byte1,msg_byte2,msg_byte3,msg_byte4,msg_byte5,msg_byte6,msg_byte7,msg_byte8,0,255,0]
        try:
            self.ser.write(bytes(buff))
        except:
            pass

    def sendGains(self,kp,kd,ki,vel_ctrl):
        kp = min(max(KP_MIN, kp), KP_MAX)
        kd = min(max(KD_MIN, kd), KD_MAX)
        ki = min(max(KI_MIN, ki), KI_MAX)
        kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
        ki_int = float_to_uint(ki, KI_MIN, KI_MAX, 12)
        msg_byte1 = 0x00 <<4 | kp_int>>8
        msg_byte2 = kp_int & 0xFF
        msg_byte3 = kd_int>>4
        msg_byte4 = (kd_int&0x0F)<<4 | ki_int>>8
        msg_byte5 = ki_int&0xFF
        msg_byte6 = 0x0
        msg_byte7 = 0x0
        msg_byte8 = vel_ctrl&0x01

        buff = [0,0,self.id,4,msg_byte1,msg_byte2,msg_byte3,msg_byte4,msg_byte5,msg_byte6,msg_byte7,msg_byte8,0,255,0]
        try:
            self.ser.write(bytes(buff))
        except:
            pass