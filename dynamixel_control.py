import serial
import time

class Dynamixel:
    def __init__(self, com, baud):
        self.str_comport = com
        self.str_baudrate = baud

    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.str_baudrate, timeout=0)

    def setReadMotorPacket(self,deviceID,Offset,Length):
        readPacket = [0xFF, 0xFF, deviceID, 0x04, 0x02, Offset, Length]
        checkSumOrdList = readPacket[2:]
        checkSumOrdListSum = sum(checkSumOrdList)
        computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
        readPacket.append(computedCheckSum)
        self.serialDevice.write(readPacket)
        #print(readPacket)

    def getMotorQueryResponse( self, deviceID, Length ):

        queryData = 0
        responsePacketSize = Length + 6
        # responsePacket = readAllData(serialDevice)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())

        if len(responsePacket) == responsePacketSize:

            print("responsePacket=", responsePacket)

            responseID = responsePacket[2]
            errorByte = responsePacket[4]

            ### python 3
            if responseID == deviceID and errorByte == 0:
                if Length == 2:
                    queryData = responsePacket[5] + 256 * responsePacket[6]
                elif Length == 1:
                    queryData = responsePacket[5]
                    # print "Return data:", queryData
            else:
                print("Error response:", responseID, errorByte)

            responsePacketStatue = True

        else:
            responsePacketStatue = False

        print("queryData=", queryData)
        return queryData, responsePacketStatue
    
    def get(self,deviceID, address, Length):

            for i in range(0,5):
                self.setReadMotorPacket(deviceID, address, Length)
                time.sleep(0.02)
                data, status = self.getMotorQueryResponse(deviceID, Length)

                if status == True:
                    break
                else:
                    print("motor ID " + str(deviceID) + "  no response " + str(i))

            return data

    def getMotorPosition(self,id):
            data = self.get(id,36,2)
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

    def setDisableMotorTorque(self,deviceID):
            Offset = 0x18
            Packet = [0xFF, 0xFF, deviceID, 0x04, 0x03, Offset, 0x00]
            checkSumOrdList = Packet[2:]
            checkSumOrdListSum = sum(checkSumOrdList)
            computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
            Packet.append(computedCheckSum)
            self.serialDevice.write(Packet)
            print(Packet)

    def setDeviceMoving(self,deviceID, deviceType, goalPos, goalSpeed, maxTorque):

            Offset = 0x1E
            Length = 6
            numberOfServo = 1
            packetLength = int((6+1)*numberOfServo+4)
            (goalSpeedL,goalSpeedH) = self.rxPacketConversion(goalSpeed)
            (maxTorqueL,maxTorqueH) = self.rxPacketConversion(maxTorque)

            syncWritePacket = [0xFF, 0xFF, 0xFE, packetLength, 0x83, Offset, Length]
            if deviceType == "Rx" or deviceType == "Ax":
                    (positionL, positionH) = self.rxPacketConversion(goalPos)
            elif deviceType == "Ex" or deviceType == "Mx":
                    (positionL, positionH) = self.exPacketConversion(goalPos)
            parameterList = [deviceID, positionL, positionH, goalSpeedL,goalSpeedH,maxTorqueL,maxTorqueH]
            for parameter in parameterList:
                    syncWritePacket.append(parameter)


            checkSumOrdList = syncWritePacket[2:]
            checkSumOrdListSum = sum(checkSumOrdList)
            computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
            syncWritePacket.append(computedCheckSum)
            self.serialDevice.write(syncWritePacket)
            print(syncWritePacket)


            #print(syncWritePacket,"goalPos =",goalPos)