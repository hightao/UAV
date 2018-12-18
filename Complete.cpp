#include <iostream>
#include <pthread.h>
#include <semaphore.h>
#include <memory.h>
#include "tcpsockets/tcpconnector.h"
#include "mobile_sample.hpp"
#include "crc16.hpp"

using namespace DJI;
using namespace DJI::OSDK;
using namespace std;
#define NUM 20
#define frame_len_ground 100
#define frame_len_sphu 100
#define CONNECT_T 0x01 //连接s-t
#define CONNECT_T_ACK 0x01
#define CONNECT_P 0x02 //连接sphu
#define CONNECT_P_ACK 0x02
#define GETSTATUS_T 0x03 //获取s-t状态
#define GETSTATUS_T_ACK 0x03
#define GETSTATUS_P 0x04 //获取sphu状态
#define GETSTATUS_P_ACK 0x04
//#define NUM 10
typedef unsigned char byte;
byte GroundCache[NUM][frame_len_ground];
byte SphuCache[NUM][frame_len_sphu];
sem_t gData_blank, gACK_product, gRELAY_product;
sem_t sData_blank, sData_product;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
bool send_ack_bool = false, relay_bool = false;
TCPStream *stream = NULL;
TCPAcceptor *acceptor = new TCPAcceptor(8888);
CRC16 crcc;

int byteToint(byte bytes)
{
    int addr = (bytes << 24) & 0xFF000000;
    return addr;
}
unsigned short bytesToshort(byte *bytes, int size = 2)
{
    unsigned short addr = (bytes[0] << 8) & 0xFF00;
    addr |= bytes[1] & 0xFF;
    return addr;
}

void intToByte(unsigned int i, byte *bytes, int size = 4)
{
    bytes[0] = (byte)(i >> 24) & 0xff;
    bytes[1] = (byte)(i >> 16) & 0xFF;
    bytes[2] = (byte)(i >> 8) & 0xFF;
    bytes[3] = (byte)(i & 0xFF);
    return;
}
void shortToByte(unsigned short i, byte *bytes, int size = 2)
{
    bytes[0] = (byte)(i >> 8) & 0xFF;
    bytes[1] = (byte)(i & 0xFF);
    return;
}
byte *gen_SPHUFrame(char *Frame, int data_length)
{
    byte SPHUFrame[98] = {0};
    //char RequestID = 1, MetaID = 2, Reserve = 3, Length = 4;
    //生成测试帧
    memcpy(SPHUFrame, &(Frame[2]), 2);       //RequestID
    memcpy(&(SPHUFrame[2]), &(Frame[3]), 1); //MetaID
    //Reserve 1 B
    memcpy(&(SPHUFrame[4]), &(Frame[7]), 1); //Length
    memcpy(&(SPHUFrame[5],&(Frame[8]),data_length);
    return SPHUFrame;
}
byte *gen_GroundFrame(byte *Frame, int i)
{
    byte GroundFrame[100] = {0}, Data[13] = {0};
    byte ack_type, length, status = 0x01, DeviceType = 0x33, Power = 0x64, FrameCount = 0x01, FrameNum = 0x01;
    byte DeviceSer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    byte Ver[2] = {0x01, 0x01};
    //char g_crc = 1, RequestID = 2, MetaID = 3, FrameCount = 4, FrameCount = 5, Length = 6;
    unsigned short ground_crc = 0;
    byte *ground_crc_byt = new byte[2];
    //生成测试帧

    memcpy(&(GroundFrame[2]), &(Frame[2]), 2); //RespondID
    switch (Frame[i] && 0xff)
    {
    case CONNECT_T_ACK:
        ack_type = CONNECT_T_ACK;
        length = 0x0d;                             //data length 13
        memcpy(Data, &status, 1);                  //Data-status
        memcpy(&(Data[1]), &DeviceType, 1);        //Data-DeviceType
        memcpy(&(Data[2]), DeviceSer, 8);          //Data-DeviceSer
        memcpy(&(Data[10]), &Power, 1);            //Data-Power
        memcpy(&(Data[11]), Ver, 2);               //Data-Ver  注意这里应该是float
        memcpy(&(GroundFrame[4]), &ack_type, 1);   //MetaID
        memcpy(&(GroundFrame[5]), &FrameCount, 1); //FrameCount
        memcpy(&(GroundFrame[6]), &FrameNum, 1);   //FrameNum
        memcpy(&(GroundFrame[7]), &length, 1);     //length
        memcpy(&(GroundFrame[8]), Data, 13);       //data
        break;
    case CONNECT_P_ACK:
        ack_type = CONNECT_P_ACK;
        length = 0x0d;                             //data length 13
        memcpy(Data, &status, 1);                  //Data-status
        memcpy(&(Data[1]), &DeviceType, 1);        //Data-DeviceType
        memcpy(&(Data[2]), DeviceSer, 8);          //Data-DeviceSer
        memcpy(&(Data[10]), &Power, 1);            //Data-Power
        memcpy(&(Data[11]), Ver, 2);               //Data-Ver  注意这里应该是float
        memcpy(&(GroundFrame[4]), &ack_type, 1);   //MetaID
        memcpy(&(GroundFrame[5]), &FrameCount, 1); //FrameCount
        memcpy(&(GroundFrame[6]), &FrameNum, 1);   //FrameNum
        memcpy(&(GroundFrame[7]), &length, 1);     //length
        memcpy(&(GroundFrame[8]), Data, 13);       //data
        memcpy(&(GroundFrame[4]), &ack_type, 1);   //MetaID
        break;
    case GETSTATUS_T_ACK:
        ack_type = GETSTATUS_T_ACK;
        memcpy(&(GroundFrame[4]), &ack_type, 1); //MetaID
        break;
    case GETSTATUS_P_ACK:
        ack_type = GETSTATUS_P_ACK;
        memcpy(&(GroundFrame[4]), &ack_type, 1); //MetaID
        break;
    default:
        break;
    }
    ground_crc = crcc.ComputeChecksum((unsigned char *)&(GroundFrame[2]), 6 + 13); //测试一下能不能直接length
    shortToByte(ground_crc, ground_crc_byt, 2);
    memcpy(GroundFrame, ground_crc_byt, 2);
    return GroundFrame;
}

void parseFromG_TCallback(Vehicle* vehicle,RecvContainer recvFrame,UserData userData))
{
    // First, lets cast the userData to LinuxSetup*
    uint8_t *mobile_data;
    int a = 0, r = NUM / 2, data_length = 0;
    unsigned short crc = 0, crc_origin = 0;
    byte data_length_by = 0;
    //time count begin
    mobile_data = reinterpret_cast<uint8_t *>(&recvFrame.recvData.raw_ack_array);
    data_length_by = mobile_data[7];
    data_length = byteToint(data_length_by);
    cout << "_____data_length:" << data_length << endl;
    crc_origin = bytesToshort(mobile_data);
    crc = crcc.ComputeChecksum(&(mobile_data[2]), 6 + data_length);
    if (mobile_data[4] && 0xff == CONNECT_T)
    {
        send_ack_bool = true;
    }
    else if (mobile_data[4] && 0xff == CONNECT_P)
    {
        relay_bool = true; //test
    }
    if (crc_origin == crc)
    {
        cout << "ground data crc right!" << endl;
    }
    else
    {
        cout << "ground data crc wrong!" << endl;
    }
    sem_wait(&gData_blank);
    pthread_mutex_lock(&lock);
    //根据标志位进行数据处理决定是send_ack or relay
    if (send_ack_bool)
    {
        send_ack_bool = false;
        GroundCache[a] = {0};
        memcpy(&(GroundCache[a]), mobile_data, 8 + data_length); //获得grounddata
        a = (++a) % (NUM / 2);
        pthread_mutex_unlock(&lock);
        sem_post(&gACK_product);
    }
    else if (relay_bool)
    {
        relay_bool = false;
        GroundCache[r] = {0}; //获得grounddata
        memcpy(&(GroundCache[a]), mobile_data, 8 + data_length);
        r = NUM / 2 + (++r) % (NUM / 2);
        pthread_mutex_unlock(&lock);
        sem_post(&gRELAY_product);
    }
}
void *recvGroundData(void *vehiclePtr)
{
    Vehicle *vehicle = (Vehicle *)vehiclePtr;
    UserData Data;
    vehicle->mobileDevice->setFromMSDKCallback(parseFromG_TCallback, Data);
}

void *sendToGround_ACK(void *vehiclePtr)
{
    Vehicle *vehicle = (Vehicle *)vehiclePtr;
    byte ACK_Frame[100];
    int i = 0;
    sem_wait(&gACK_product);
    pthread_mutex_lock(&lock);

    //取出相应的帧，决定ACK类型
    memcpy(ACK_Frame, gen_GroundFrame(GroundCache[i], 4), 100);
    i = (++i) % (NUM / 2);
    vehicle->mobileDevice->sendDataToMSDK(reinterpret_cast<uint8_t *>(ACK_Frame), 100);
    pthread_mutex_unlock(&lock);
    sem_post(&gData_blank);
}

void *relay(void *vehiclePtr)
{
    Vehicle *vehicle = (Vehicle *)vehiclePtr;
    int i = 0;
    byte GROUND_Frame[98] = {0}; //在之后多帧时，需要改成动态分配
    int data_length = 0;
    byte data_length_by = 0;
    while (1)
    {
        sem_wait(&gRELAY_product);
        pthread_mutex_lock(&lock);
        //取出相应的帧，relay 给sphu
        data_length_by = GroundCache[i][7];
        data_length = byteToint(data_length_by);
        memcpy(GROUND_Frame, gen_SPHUFrame(GroundCache[i], data_length), 6 + data_length);
        stream->send(GROUND_Frame, 98);
        i = (NUM / 2) + (++i) % (NUM / 2);
        pthread_mutex_unlock(&lock);
        sem_post(&gData_blank);
    }
}

void *recvSphuData(void *vehiclePtr)
{
    Vehicle *vehicle = (Vehicle *)vehiclePtr;
    int i = 0;
    while (1)
    {
        if (stream != NULL)
        {
            ssize_t len;
            char line[100] = {0};
            while ((len = stream->receive(line, sizeof(line))) > 0)
            {
                sem_wait(sData_blank);
                SphuCache[i] = {0}; //获取sphu Data
                memcpy(&(SphuCache[i]), line, len);
                gen_GroundFrame(SphuCache[i], 2);
                i = (++i) % (NUM);
                sem_post(sData_product);
                // line[len] = 0;
                // printf("received - %s\n", line);
            }
            // delete stream;
        }
    }
}

void *sendToGround_SPHU(void *vehiclePtr)
{
    Vehicle *vehicle = (Vehicle *)vehiclePtr;
    char SPHU_Frame[100];
    int i = 0;
    while (1)
    {
        sem_wait(sData_product);
        pthread_mutex_lock(&lock);
        //取出SPHU Data
        memcpy(SPHU_Frame, gen_GroundFrame(&(SphuCache[i]), 2), 100);
        //SphuCache[i] = ;
        i = (++i) % (NUM);
        //封装一下扔回地面
        vehicle->mobileDevice->sendDataToMSDK(reinterpret_cast<uint8_t *>(SPHU_Frame), 100);
        pthread_mutex_unlock(&lock);
        sem_post(sData_blank);
    }
}

int main(int argc, char **argv)
{

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle *vehicle = linuxEnvironment.getVehicle();
    if (vehicle == nullptr)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }
    if (acceptor->start() != 0)
    {
        cout << "TCP connect error!" << endl;
    }
    stream = acceptor->accept();

    pthread_t pid_recvground, pid_relay, pid_sendground_ack, pid_recvsphu, pid_sendground_sphu;
    sem_init(&gData_blank, 0, NUM);
    sem_init(&gACK_product, 0, 0);
    sem_init(&gRELAY_product, 0, 0);
    sem_init(&sData_blank, 0, NUM);
    sem_init(&sData_product, 0, 0);

    pthread_create(&pid_recvground, NULL, recvGroundData, (void *)vehicle);
    pthread_create(&pid_relay, NULL, relay, (void *)vehicle);
    pthread_create(&pid_sendground_ack, NULL, sendToGround_ACK, (void *)vehicle);
    pthread_create(&pid_recvsphu, NULL, recvSphuData, (void *)vehicle);
    pthread_create(&pid_sendground_sphu, NULL, sendToGround_SPHU, (void *)vehicle);

    pthread_join(pid_recvground, NULL);
    pthread_join(pid_relay, NULL);
    pthread_join(pid_sendground_ack, NULL);
    pthread_join(pid_recvsphu, NULL);
    pthread_join(pid_sendground_sphu, NULL);

    sem_destroy(&gData_blank);
    sem_destroy(&sData_blank);
    sem_destroy(&gACK_product);
    sem_destroy(&gRELAY_product);
    sem_destroy(&sData_product);

    std::string input = "";

    // How to get a string/sentence with spaces
    std::cout << "Please choose the test :\n 1 means testing moc \n 2 means testing mobileDevice\n 3 means testing both of them\n"
              << std::endl;

    // How to get a number.
    int myNumber = 0;

    while (true)
    {
        std::cout << "Please enter a valid number: ";
        getline(std::cin, input);

        // This code converts from string to number safely.
        std::stringstream myStream(input);
        if (myStream >> myNumber && myNumber >= 1 && myNumber <= 3)
            break;
        std::cout << "Invalid number, please try again" << std::endl;
    }
    std::cout << "You entered: " << myNumber << std::endl
              << std::endl;
    setTestSuite(myNumber);
    setupMSDKParsing(vehicle, &linuxEnvironment);

    return 0;
}
