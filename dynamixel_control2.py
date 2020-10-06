import serial
import time

class Dynamixel:
    def __init__(self, com, baud):
        self.str_comport = com
        self.baudrate = baud
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("Initial Dynamixel : comport = " + self.str_comport + " , baudrate = " + str(self.baudrate))
        try:
            self.connect()
            print("Dynamixel initial successfully + + +")
        except Exception as e:
            print(e)
            print("Dynamixel initial fail!!")

    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.baudrate, timeout=0)



    def setWritePackage(self,ID,address,bytes,data):
        length = bytes + 2 + 3
        TxBuf = [0xff, 0xff, 0xfd, 0x00, ID, length & 0xff, (length >> 8) & 0xff, 0x03, address & 0xff, (address >>8) & 0xff]
        #print("byte =",bytes, type(bytes))
        for i in range(0,bytes):
            
            TxBuf.append(data[i])
        
        CRC = self.updateCRC(0, TxBuf, len(TxBuf))
        TxBuf.append(CRC & 0x00FF)
        TxBuf.append((CRC >> 8) & 0x00FF)

        try:
            self.serialDevice.write(TxBuf)
        except:
            print("Serial Error!! [setWriteMotorPacket]")
        #print(TxBuf)

        

    def setReadMotorPacket(self,deviceID,Address,Length):
        #package = H1 H2 H3 RSRV ID LEN1 LEN2 INST PARAM1 PARAM2 PARAM3 PARAM4 CRC1 CRC2
        readPackage = [0xFF, 0xFF, 0xFD, 0x00, deviceID, 0x07, 0x00, 0x02, Address & 0xff, (Address >> 8) &0xff, Length & 0xff, (Length >> 8) &0xff]
        CRC = self.updateCRC(0, readPackage, len(readPackage))
        readPackage.append(CRC & 0x00FF)
        readPackage.append((CRC >> 8) & 0x00FF)

        #print("ReadPackage = ", [hex(x) for x in readPackage])

       
        try:
            self.serialDevice.write(readPackage)
        except:
            print("Serial Error!! [setReadMotorPacket]")
        #print(readPackage)

    def getMotorQueryResponse( self, deviceID ):

            queryData = 0
            responsePacketSize = 15

            responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())

            #print("len responsePacket = ", len(responsePacket))
            #print("responsePacket",responsePacket)

            if len(responsePacket) == responsePacketSize:

                #print("responsePacket=", responsePacket)

                responseID = responsePacket[4]
                errorByte = responsePacket[8]

                ### python 3
                if responseID == deviceID and errorByte == 0:
                    queryData = responsePacket[9] + 256 * responsePacket[10]

                else:
                    pass
                    #print("Error response:", responseID, errorByte)

                responsePacketStatue = True

            else:
                responsePacketStatue = False

            #print("queryData=", queryData)
            return queryData,responsePacketStatue

    def get(self,deviceID, address, Length):

            for i in range(0,5):
                self.setReadMotorPacket(deviceID, address, Length)
                time.sleep(0.02)
                data, status = self.getMotorQueryResponse(deviceID)

                if status == True:
                    break
                else:
                    pass
                    print("motor ID " + str(deviceID) + "  no response " + str(i))

            return data

    def getMotorPosition(self,id):
            data = self.get(id,0x84,4)
            #print("motor ",id,"position",data)
            return data
    
    def getMotorVelocity(self,id):
            data = self.get(id,0x68,4)
            #print("motor ",id,"velocity",data)
            return data

    def rxPacketConversion( self,value ):
            if value < 1024 and value >= 0:
                    hiByte = int(value/256)
                    loByte = value%256
            else:
                    print("rxPacketConversion: value out of range", value)
            return loByte, hiByte

    def exPacketConversion( self,value ):
            if value < 4096 and value >= 0:
                    hiByte = int(value/256)
                    loByte = value%256
            else:
                    print("exPacketConversion: value out of range", value)
            return loByte, hiByte

    def setStatusReturnLevel(self,status):
        return_status = [status]
        for i in range(0,50):
            self.setWritePackage(i ,0x44,1,return_status)
        #time.sleep(0.01)

    def setDisableMotorTorque(self,deviceID):
        torque_status = [0]
        self.setWritePackage(deviceID ,0x40,1,torque_status)
        #time.sleep(0.01)
    def setEnableMotorTurque(self,deviceID):
        torque_status = [1]
        self.setWritePackage(deviceID ,0x40,1,torque_status)
    


    def setDeviceMoving( self,deviceID, deviceType, goalPos, goalSpeed, maxTorque):
        #print("goal pos = ",goalPos)
        position = [goalPos & 0xff, (goalPos >> 8) & 0xff, 0x00, 0x00]
        self.setWritePackage(deviceID ,0x74,4,position)


    def setDeviceVelocity(self, deviceID, goalSpeed):
        speed = [goalSpeed & 0xff, (goalSpeed >> 8) & 0xff, 0x00, 0x00]
        self.setWritePackage(deviceID ,0x68,4,speed)


    def InterpolateMotorValue(self,finish_value,start_value,finish_time,start_time,current_time):
        motor_value = int((finish_value - start_value)*(current_time-start_time)/(finish_time - start_time)+start_value)
        return motor_value

    def updateCRC(self, crc_accum, data_blk_ptr, data_blk_size):
        crc_table = [0x0000,
                     0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                     0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                     0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                     0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                     0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                     0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                     0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                     0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                     0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                     0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                     0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                     0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                     0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                     0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                     0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                     0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                     0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                     0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                     0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                     0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                     0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                     0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                     0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                     0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                     0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                     0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                     0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                     0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                     0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                     0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                     0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                     0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                     0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                     0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                     0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                     0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                     0x820D, 0x8207, 0x0202]

        for j in range(0, data_blk_size):
            i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF
            crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF

        return crc_accum