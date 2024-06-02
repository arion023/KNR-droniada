# -*- coding:utf-8 -*-

'''!
  @file DFRobot_TMF8x01.py
  @brief 定义DFRobot_TMF8x01 类的基础结构，基础方法的实现。实现测距功能，支持TMF8801和TMF8701类型的TOF测距传感器。
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      Arya(xue.peng@dfrobot.com)
  @version     V1.0
  @date        2021-03-16
  @url https://github.com/DFRobot/DFRobot_TMF8x01
'''

import sys
import time
import smbus
import RPi.GPIO as GPIO
import Drv_TMF8x01 as drv

class DFRobot_TMF8x01:
  ePROXIMITY = 0
  eDISTANCE = 1
  eCOMBINE = 2
  
  _addr = 0x41
  _initialize = False
  _STEPPER_COUNT = 1
  _MOTOR_COUNT = 2
  _cpu = 600000
  _count = 0
  _host = [0,0,0,0,0]
  _module = [0,0,0,0,0]
  _distance = []
  _timestamp = 0
  _tid = 0
  _calib_data = []
  _algo_state_data = []
  _measure_cmd_set = []
  _measure_cmd_flag = False
  
  result_dictKey = ['status', 'regContents','tid','resultNumber','resultInfo','disL','disH','syscolck0','syscolck1','syscolck2','syscolck3']
  result_dict = {}
  
  TMF8801_CALIB_DATA = [0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04]
  TMF8801_ALGO_STATE = [0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
  TMF8x01_MEASUREMENT_DATA = []
  
  ## Enum calibration mode 
  eMODE_NO_CALIB = 0
  eMODE_CALIB  = 1
  eMODE_CALIB_AND_ALGOSTATE = 3
  
  REG_MTF8x01_ENABLE = 0xE0
  REG_MTF8x01_APPID = 0x00
  REG_MTF8x01_ID = 0xE3
  REG_MTF8x01_APPREQID = 0x02
  
  REG_MTF8x01_CMD_DATA9 = 0X06
  REG_MTF8x01_CMD_DATA8 = 0X07
  REG_MTF8x01_CMD_DATA7 = 0X08
  REG_MTF8x01_CMD_DATA6 = 0X09
  REG_MTF8x01_CMD_DATA5 = 0X0A
  REG_MTF8x01_CMD_DATA4 = 0X0B
  REG_MTF8x01_CMD_DATA3 = 0X0C
  REG_MTF8x01_CMD_DATA2 = 0X0D
  REG_MTF8x01_CMD_DATA1 = 0X0E
  REG_MTF8x01_CMD_DATA0 = 0X0F
  REG_MTF8x01_COMMAND = 0X10
  REG_MTF8x01_FACTORYCALIB = 0X20
  REG_MTF8x01_STATEDATAWR = 0X2E
  REG_MTF8x01_STATUS = 0x1D
  REG_MTF8x01_CONTENTS = 0x1E
  REG_MTF8x01_RESULT_NUMBER = 0x20
  REG_MTF8x01_VERSION_MAJOR = 0X01
  REG_MTF8x01_VERSION_MINORANDPATCH = 0x12
  REG_MTF8x01_VERSION_HW = 0xE3
  REG_MTF8x01_VERSION_SERIALNUM = 0x28
  REG_MTF8x01_INT_ENAB = 0xE2
  REG_MTF8x01_INT_STATUS = 0xE1
  REG_MTF8x01_TJ         = 0x32
  
  SENSOR_MTF8x01_CALIBRATION_SIZE = 14
  
  MODEL_TMF8801 = 0x4120
  MODEL_TMF8701 = 0x5e10
  
  ## Enum CMDSET 
  CMDSET_INDEX_CMD6 = 1
  CMDSET_BIT_PROXIMITY = 0
  CMDSET_BIT_DISTANCE = 1
  CMDSET_BIT_INT = 4
  CMDSET_BIT_COMBINE = 5
  
  CMDSET_INDEX_CMD7 = 0
  CMDSET_BIT_CALIB = 0
  CMDSET_BIT_ALGO = 1
  
  ## Board status 
  STA_OK = 0x00
  STA_ERR = 0x01
  STA_ERR_DEVICE_NOT_DETECTED = 0x02
  STA_ERR_SOFT_VERSION = 0x03
  STA_ERR_PARAMETER = 0x04

  ## last operate status, users can use this variable to determine the result of a function call. 
  last_operate_status = STA_OK
  
  def __init__(self, enPin, intPin, bus_id):
    self._en = enPin
    self._intPin = intPin
    self._bus = smbus.SMBus(bus_id)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

  def begin(self):
    '''!
      @brief    initialization sensor's interface, addr, ram config to running APP0 application.
      @return   initialization sucess return 0, fail return -1
    '''
    self._initialize = False
    if(self._en > -1):
      GPIO.setup(self._en, GPIO.OUT)
      GPIO.output(self._en, GPIO.LOW)
      time.sleep(1)
      GPIO.output(self._en, GPIO.HIGH)
      time.sleep(1)
    if(self._intPin > -1):
      GPIO.setup(self._intPin, GPIO.IN, GPIO.PUD_UP)
      
    self.sleep()
    self._write_bytes(self.REG_MTF8x01_ENABLE, [0x01])
    if(self._wait_for_cpu_ready() != True):
      print("_wait_for_cpu_ready is failed.")
      return -1
    if(self._get_app_id() == 0x80):
      #print("bootloader")
      if(self._download_ram_patch() != True):
        print("download ram patch is failed.")
        return -1
      if (self._waitForApplication() != True):
        self._load_application()
        if(self._waitForApplication() != True):
          print("APP0 is not running.")
          return -1
    self._initialize = True
    return 0

  def sleep(self):
    '''!
      @brief  sleep sensor by software, the sensor enter sleep mode(bootloader). Need to call wakeup function to wakeup sensor to enter APP0
    '''
    rslt = self._read_bytes(self.REG_MTF8x01_ENABLE,1)
    rslt[0] = rslt[0] |  (1 << 7)
    self._write_bytes(self.REG_MTF8x01_ENABLE, rslt)
    self._measure_cmd_flag = False
    self._count = 0
  
  def wakeup(self):
    '''!
      @brief  wakeup device from sleep mode, it will running app0.
      @return enter app0 return true, or return false.
    '''
    self._write_bytes(self.REG_MTF8x01_ENABLE, [0x01])
    if(self._wait_for_cpu_ready() != True):
      return False
    if(self._get_app_id() == 0x80):
      #print("bootloader")
      if(self._download_ram_patch() != True):
        return False
      if (self._waitForApplication() != True):
        return False
    #else:
      #print("app0")
    if(self._measure_cmd_set[self.CMDSET_INDEX_CMD6] & (1<< self.CMDSET_BIT_INT)):
      self.modifyCmdSet(self.CMDSET_INDEX_CMD6, self.CMDSET_BIT_INT, True)
    if(self._set_caibration_mode(self._get_calibration_mode()) == False):
      return False
    return True

  def get_unique_id(self): 
    '''!
      @brief get a unique number of sensor .Each sensor has a unique identifier.
      @return return 4bytes unique number:
      @n  the byte0 of return: serial_number_0
      @n  the byte1 of return: serial_number_1
      @n  the byte2 of return: identification_number_1
      @n  the byte2 of return: identification_number_0
    '''
    self._write_bytes(self.REG_MTF8x01_COMMAND, [0x47])
    rslt = [0]
    waitForTimeOutMs = 0.1
    waitForTimeoutIncMs = 0.005
    t = 0
    while t < waitForTimeOutMs:
      time.sleep(waitForTimeoutIncMs)
      rslt = self._read_bytes(self.REG_MTF8x01_CONTENTS, 1)
      if rslt[0] == 0x47:
        rslt = self._read_bytes(self.REG_MTF8x01_VERSION_SERIALNUM, 4)
        r = rslt[3] << 24 | rslt[2] << 16 | rslt[1] << 8 | rslt[0]
        self._stop_command()
        return r
    return 0

  def get_sensor_model(self): 
    '''!
      @brief get sensor's model.
      @return return a String:
      @n  TMF8801: the sensor is TMF8801
      @n  TMF8701: the sensor is TMF8701
      @n  unknown : unknown device
    '''
    str1 = "unknown"
    rslt = self.get_unique_id()
    rslt = (rslt >> 16)&0xFFFF
    if(rslt == self.MODEL_TMF8801):
      str1 = "TMF8801"
    elif (rslt == self.MODEL_TMF8701):
      str1 = "TMF8701"
    return str1

  def get_software_version(self): 
    '''!
      @brief get software version of patch.
      @return return string of device software version,format:
      @n major.minor.patch numbers.chip id version
    '''
    str1 = ""
    rslt = self._read_bytes(self.REG_MTF8x01_VERSION_MAJOR, 1)
    lin = "%X"%rslt[0]
    str1 =str1 + lin +'.'
    rslt = self._read_bytes(0x12, 1)
    lin = "%X"%rslt[0]
    #print("%#x"%rslt[0])
    str1 =str1 + lin +'.'
    rslt = self._read_bytes(0x13, 1)
    lin = "%X"%rslt[0]
    #print("%#x"%rslt[0])
    str1 =str1 + lin +'.'
    rslt = self._read_bytes(0xe4, 1)
    lin = "%X"%rslt[0]
    #print("%#x"%rslt[0])
    str1 =str1 + lin
    return str1

  def get_calibration_data(self): 
    '''!
      @brief  Get 14 bytes of calibration data.
      @return return 14 bytes of calibration data.
    '''
    #print("get_calibration_data %d"%self._initialize)
    rslt = []
    if(self._initialize == False):
      #print(self._initialize)
      return list()
    time.sleep(0.0001)
    self._write_bytes(self.REG_MTF8x01_COMMAND, [0x0a])
    time.sleep(0.0001)
    if self._checkStatusRegister(0x0a) == True:
      time.sleep(0.0001)
      rslt = self._read_bytes(self.REG_MTF8x01_RESULT_NUMBER, 14)
      print(rslt)
      self._stop_command()
      return rslt
    else:
      return list()

  def set_calibration_data(self, l): 
    '''!
      @brief  set 14 bytes of calibration data.
      @param l The list of 14 bytes calibration data.
      @return set sucess return true, or return false.
    '''
    if len(l) != 14:
      return False
    TMF8801_CALIB_DATA = l
    return True

  def _set_caibration_mode(self, calib_m = eMODE_CALIB): 
    if self._initialize == False:
      print("self._initialize == False")
      return
    if self._measure_cmd_flag == True:
      print("self._measure_cmd_flag == True")
      return
    if calib_m == self.eMODE_CALIB:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_CALIB, True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_ALGO, False)
      self._write_bytes(self.REG_MTF8x01_COMMAND, [0x0B])
      self._write_bytes(self.REG_MTF8x01_RESULT_NUMBER, self._calib_data)
    elif calib_m == self.eMODE_CALIB_AND_ALGOSTATE:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_CALIB, True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_ALGO, True)
      self._write_bytes(self.REG_MTF8x01_COMMAND, [0x0B])
      self._write_bytes(self.REG_MTF8x01_RESULT_NUMBER, self._calib_data)
      self._write_bytes(self.REG_MTF8x01_STATEDATAWR, self._algo_state_data)
    else:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_CALIB, False)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD7,self.CMDSET_BIT_ALGO, False)
    self._write_bytes(self.REG_MTF8x01_CMD_DATA7, self._measure_cmd_set)
    
    time.sleep(0.5)
    #print("start_measurement end")
    if(self._checkStatusRegister(0x55) != True):
      return False
    while self._count < 4:
      if(self.is_data_ready()): 
        self.get_distance_mm()
      time.sleep(0.002)
    self._measure_cmd_flag = True
    return True
    

  def stop_measurement(self):
    '''!
      @brief  disable measurement config.
    '''
    self._measure_cmd_flag = False
    self._write_bytes(self.REG_MTF8x01_COMMAND, [0xff])
    time.sleep(0.05)#s 50ms
    self._measure_cmd_flag = False
    self._count = 0
    self._timestamp = 1
    self._tid = 0

  def is_data_ready(self):
    '''!
      @brief  Waiting for data ready.
      @return if data is valid, return true, or return false.
    '''
    tid = 0
    reliab = 0
    t = 0
    count = 0
    i = 0
    #time.sleep(0.01)
    t = time.time()
    self.result_dict = dict(zip(self.result_dictKey, self._read_bytes(self.REG_MTF8x01_STATUS, 11)))
    if self.result_dict['regContents'] == 0x55:
      if self.result_dict['tid'] != self._tid:
        self._tid = self.result_dict['tid']
        j = self.result_dict["syscolck3"] << 24 | self.result_dict["syscolck2"] << 16 | self.result_dict["syscolck2"] << 8 | self.result_dict["syscolck0"]
        if self._count < 4:
          self._host[self._count] = t*10000
          self._module[self._count] = (j*0.2)/100
          self._count =self._count + 1
        elif self._count == 4:
          self._host[self._count] = t*10000
          self._module[self._count] = (j*0.2)/100
          t1 = self._host[4] - self._host[0]
          t2 = self._module[4] - self._module[0]
          self._timestamp = 1
          if t2 > 0:
             self._timestamp = t1/t2
          #print(self._timestamp)
          while i < 4:
            self._host[i] = self._host[i+1]
            self._module[i] = self._module[i+1]
            i = i+1
          return True
        else:
          self._count = 0
    if self._measure_cmd_set[self.CMDSET_INDEX_CMD6] & (1<<self.CMDSET_BIT_INT):
      #print("+++++++++++++++++")
      rslt = self._read_bytes(self.REG_MTF8x01_INT_STATUS,1)
      if(rslt[0] & 0x01) == 1:
        rslt[0] |= 0x01
        self._write_bytes(self.REG_MTF8x01_INT_STATUS,rslt)
      
    return False

  def get_distance_mm(self):
    '''!
      @brief  get distance, unit mm. Before using this function, you need to call is_data_ready.
      @return return distance value, unit mm.
    '''
    rslt = self.result_dict["disH"] << 8 | self.result_dict["disL"]
    dis = rslt * self._timestamp
    if self._measure_cmd_set[self.CMDSET_INDEX_CMD6] & (1<<self.CMDSET_BIT_INT):
      #print("+++++++++++++++++")
      rslt = self._read_bytes(self.REG_MTF8x01_INT_STATUS,1)
      rslt[0] |= 0x01
      self._write_bytes(self.REG_MTF8x01_INT_STATUS,rslt)
    return int(dis)

  def enable_int_pin(self):
    '''!
      @brief  enable INT pin. If you call this function,which will report a interrupt.
      @n signal to host by INT pin when measure data is ready.
    '''
    self._write_bytes(self.REG_MTF8x01_INT_ENAB,[0x01])
    self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_INT, True)

  def disable_int_pin(self):
    '''!
      @brief disable INT pin.
    '''
    self._write_bytes(self.REG_MTF8x01_INT_ENAB,[0])
    self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_INT, False)

  def power_on(self):
    '''!
      @brief power on sensor when power down sensor by EN pin.
      @return sucess return True, or return False
    '''
    if self._initialize == False:
      return False
    if (self._en < 0):
      return False

    time.sleep(1)
    GPIO.output(self._en, GPIO.HIGH)
    time.sleep(1)
  
    self._write_bytes(self.REG_MTF8x01_ENABLE, [0x01])
    if(self._wait_for_cpu_ready() != True):
      return False
    if(self._get_app_id() == 0x80):
      self._load_application()
      if self._waitForApplication() == False:
        return False
    return True

  def power_down(self):
    '''!
      @brief power down sensor by EN pin.
      @return sucess return True, or return False
    '''
    if self._initialize == False:
      return False
    if (self._en < 0):
      return False
    time.sleep(1)
    GPIO.output(self._en, GPIO.LOW)
    time.sleep(1)
    self._addr = 0x41
    return True

  def get_i2c_address(self):
    '''!
      @brief get I2C address.
      @return return 7 bits I2C address
    '''
    return self._addr

  def get_junction_temperature_C(self):
    '''!
      @brief get junction temperature of sensor.
      @return Junction temperature of sensor, unit, Celsius.
    '''
    temp = self._read_bytes(self.REG_MTF8x01_TJ, 1)

    if temp[0] & 0x80:
      return -((~temp[0]) + 1)
    else:
      return temp[0]
    

  def _download_ram_patch(self):
    pass

  def _get_calibration_mode(self):
    mode = 0
    if(self._measure_cmd_set[self.CMDSET_INDEX_CMD7] & (1<< self.CMDSET_BIT_CALIB)):
      mode = 1
    if(self._measure_cmd_set[self.CMDSET_INDEX_CMD7] & (1<< self.CMDSET_BIT_ALGO)):
      mode |= (1<<self.CMDSET_BIT_ALGO)
    return mode

  def _load_application(self):
    '''!
       @brief set cpu to run APP0.
       @return if APP0 is running return True, or return False
    '''
    self._write_bytes(self.REG_MTF8x01_APPREQID,[0xC0])
    if self._waitForApplication() == True:
      return True
    else:
      return False

  def _load_bootloader(self):
    '''!
       @brief set cpu to run Bootloader.
       @return if Bootloader is running return True, or return False
    '''
    self._write_bytes(self.REG_MTF8x01_APPREQID,[0x80])
    if self._wait_for_bootloader() == True:
      return True
    else:
      return False

  def _waitForApplication(self):
    '''!
       @brief Waiting for APP0 is running.
       @return APP0 is running in 100 ms return True, or return False
    '''
    waitForTimeOutMs = 0.1
    waitForTimeoutIncMs = 0.005
    t = 0
    #print("_waitForApplication")
    #print(t)
    while t < waitForTimeOutMs:
      time.sleep(waitForTimeoutIncMs)
      if(self._get_app_id() == 0xC0):
        return True
      t += waitForTimeoutIncMs
    return False

  def _wait_for_bootloader(self):
    '''!
       @brief Waiting for Bootloader is running.
       @return Bootloader is running in 100 ms return True, or return False
    '''
    waitForTimeOutMs = 0.1
    waitForTimeoutIncMs = 0.005
    t = 0
    while t < waitForTimeOutMs:
      time.sleep(waitForTimeoutIncMs)
      if(self._get_app_id() == 0x80):
        return True
      t += waitForTimeoutIncMs
    return False

  def _wait_for_cpu_ready(self):
    '''!
       @brief Waiting for CPU is ready.
       @return CPU is ready in 100 ms return True, or return False
    '''
    waitForTimeOutMs = 0.1
    waitForTimeoutIncMs = 0.005
    t = 0
    while t < waitForTimeOutMs:
      time.sleep(waitForTimeoutIncMs)
      if(self._get_cpu_state() == 0x41):
        return True
      t += waitForTimeoutIncMs
    return False

  def _modify_cmd_set(self, index, bit, val):
    '''!
      @brief modify array measurement cmd, _measure_cmd_set.
      @param index The index of array _measure_cmd_set,the range is 0~8
      @param bit The bit of one byte, the range is 0~7
      @param val True or False. The val of one byte's bit,false set bit to 0, true set bit to 1.
    '''
    if ((index > (len(self._measure_cmd_set) - 1)) or (bit > 7)):
      return
    if val == True:
      self._measure_cmd_set[index] |= (1 << bit)
    else:
      self._measure_cmd_set[index] = ~((~self._measure_cmd_set[index]) | (1 << bit))

  def _get_cpu_state(self):
    rslt = self._read_bytes(self.REG_MTF8x01_ENABLE,1)
    return rslt[0]

  def _get_app_id(self):
    rslt = self._read_bytes(self.REG_MTF8x01_APPID,1)
    return rslt[0]

  def _cal_check_sum(self, l):
    sum = 0
    for i in l:
      sum += i
    sum = sum ^ 0xff
    return sum

  def _read_status_ack(self):
    rslt = self._read_bytes(0x08,3)
    s = rslt[0] | (rslt[1] << 8) | (rslt[2] << 16)
    if(s == 0xFF0000):
      return True
    
    return False

  def _stop_command(self):
    '''!
      @brief  stop command.
    '''
    self._write_bytes(self.REG_MTF8x01_COMMAND, [0xff])
    time.sleep(0.1)#s 50ms

  def _checkStatusRegister(self, value):
    waitForTimeOutMs = 1
    waitForTimeoutIncMs = 0.005
    t = 0
    #print(t)
    while t < waitForTimeOutMs:
      #print(t)
      time.sleep(waitForTimeoutIncMs)
      t += waitForTimeoutIncMs
      rslt = self._read_bytes(self.REG_MTF8x01_CONTENTS, 1)
      #print("%#x"%rslt[0])
      if rslt[0] == value:
        return True
    return False
  
  def _write_bytes(self, reg, buf):
    self.last_operate_status = self.STA_ERR_DEVICE_NOT_DETECTED
    try:
      self._bus.write_i2c_block_data(self._addr, reg, buf)
      self.last_operate_status = self.STA_OK
    except:
      pass

  def _read_bytes(self, reg, len1):
    self.last_operate_status = self.STA_ERR_DEVICE_NOT_DETECTED
    try:
      rslt = self._bus.read_i2c_block_data(self._addr, reg, len1)
      self.last_operate_status = self.STA_OK
      return rslt
    except:
      return [0] * len1




class DFRobot_TMF8801(DFRobot_TMF8x01):
  def __init__(self,enPin = -1, intPin = -1, bus_id = 1):
    self._calib_data = [0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04]
    self._algo_state_data = [0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    self._measure_cmd_set = [0x01, 0xA3, 0x00, 0x00,0x00, 0x64, 0x03, 0x84, 0x02]
    DFRobot_TMF8x01.__init__(self,enPin,intPin, bus_id)
  
  def start_measurement(self, calib_m):
    '''!
    @brief Config measurement params to enable measurement. Need to call stop_measurement to stop ranging action.
    @param calib_m: Is an enumerated variable, which is to config measurement cailibration mode.
    @n     eMODE_NO_CALIB  :          Measuring without any calibration data.
    @n     eMODE_CALIB    :          Measuring with calibration data.
    @n     eMODE_CALIB_AND_ALGOSTATE : Measuring with calibration and algorithm state.
    @return status:
    @n      false:  enable measurement failed.
    @n      true:  enable measurement sucess.
    '''
    return self._set_caibration_mode(calib_m)
  
  def _download_ram_patch(self):
    '''
      @brief  download RAM patch.
      @return download sucess return True, or return False
    '''
    if self._get_app_id() != 0x80:
      if(self._load_bootloader() != True):
        print("load Bootloader failed")
        return False
    bufList = [0x08,0x14,0x01,0x29]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._read_status_ack() != True):
      return False
    
    bufList = [0x08,0x43,0x02,0x00,0x00]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._read_status_ack() != True):
      return False
    
    
    l = drv.DFRobot_TMF8801_initBuf
    i = 0
    flag = 0
    while l[i] > 0:
      bufList = [0x08,0x41]
      flag = l[i]
      bufList.append(flag)
      i = i + 1
      bufList = bufList + l[i:i+flag]
      i = i + flag
      bufList.append(self._cal_check_sum(bufList[1:]))
      self._write_bytes(bufList[0],bufList[1:])
      if(self._read_status_ack() != True):
        return False
    bufList = [0x08,0x11,0x00]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._wait_for_cpu_ready()):
      return True
    return False
  
class DFRobot_TMF8701(DFRobot_TMF8x01):
  def __init__(self,enPin = -1, intPin = -1, bus_id = 1):
    self._calib_data = [0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04]
    self._algo_state_data = [0xB1, 0xA9, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    self._measure_cmd_set = [0x01, 0xA3, 0x00, 0x00,0x00, 0x64, 0x03, 0x84, 0x02]
    DFRobot_TMF8x01.__init__(self,enPin, intPin, bus_id)

  def start_measurement(self, calib_m, mode):
    '''
    @brief Config measurement params to enable measurement. Need to call stop_measurement to stop ranging action.
    @param calib_m: Is an enumerated variable , which is to config measurement cailibration mode.
    @n     eMODE_NO_CALIB  :          Measuring without any calibration data.
    @n     eMODE_CALIB    :          Measuring with calibration data.
    @n     eMODE_CALIB_AND_ALGOSTATE : Measuring with calibration and algorithm state.
    @param mode : the ranging mode of TMF8701 sensor.
    @n     ePROXIMITY: Raing in PROXIMITY mode,ranging range 0~10cm
    @n     eDISTANCE: Raing in distance mode,ranging range 10~60cm
    @n     eCOMBINE:  Raing in PROXIMITY and DISTANCE hybrid mode,ranging range 0~60cm
    @return status:
    @n      false:  enable measurement failed.
    @n      true:  enable measurement sucess.
    '''
    if mode == self.ePROXIMITY:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_PROXIMITY,True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_DISTANCE,False)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_COMBINE,False)
    elif mode == self.eDISTANCE:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_PROXIMITY,False)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_DISTANCE,True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_COMBINE,False)
    elif mode == self.eCOMBINE:
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_PROXIMITY,True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_DISTANCE,True)
      self._modify_cmd_set(self.CMDSET_INDEX_CMD6,self.CMDSET_BIT_COMBINE,True)
    return self._set_caibration_mode(calib_m)

  def _download_ram_patch(self):
    '''!
      @brief  download RAM patch.
      @return download sucess return True, or return False
    '''
    if self._get_app_id() != 0x80:
      if(self._load_bootloader() != True):
        print("load Bootloader failed")
        return False
    bufList = [0x08,0x14,0x01,0x29]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._read_status_ack() != True):
      return False
    
    bufList = [0x08,0x43,0x02,0x00,0x00]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._read_status_ack() != True):
      return False
    
    
    l = drv.DFRobot_TMF8801_initBuf
    i = 0
    flag = 0
    while l[i] > 0:
      bufList = [0x08,0x41]
      flag = l[i]
      bufList.append(flag)
      i = i + 1
      bufList = bufList + l[i:i+flag]
      i = i + flag
      bufList.append(self._cal_check_sum(bufList[1:]))
      self._write_bytes(bufList[0],bufList[1:])
      if(self._read_status_ack() != True):
        return False
    bufList = [0x08,0x11,0x00]
    bufList.append(self._cal_check_sum(bufList[1:]))
    self._write_bytes(bufList[0],bufList[1:])
    if(self._wait_for_cpu_ready()):
      return True
    return False
