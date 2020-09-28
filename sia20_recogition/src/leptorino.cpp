// =============================================================================
//	CFS_Sample 本体部
//
//					Filename: main.c
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Gpop/Series.hpp>
#include "/home/robot/catkin_ws/src/sia20/sia20_control/src/cliping_leptorino.hpp"

#include <rs_comm/pCommon.h>
#include <rs_comm/rs_comm.h>
#include <rs_comm/pComResInternal.h>

// =============================================================================
//	マクロ定義
// =============================================================================
#define PRG_VER	"Ver 1.0.0"

// =============================================================================
//	構造体定義
// =============================================================================
typedef struct ST_SystemInfo {
	int com_ok;
} SystemInfo;

// =============================================================================
//	プロトタイプ宣言
// =============================================================================
void App_Init(void);
void App_Close(void);
int GetRcv_to_Cmd( char *rcv, char *prm);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(void);
void SerialStart(void);
void SerialStop(void);

// =============================================================================
//	モジュール変数定義
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];

// ----------------------------------------------------------------------------------
//	メイン関数
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	int i, l = 0, rt = 0;
	int mode_step = 0;
	int AdFlg = 0, EndF = 0;
	long cnt = 0;
	UCHAR strprm[256];
	ST_RES_HEAD *stCmdHead;
	ST_R_DATA_GET_F *stForce;
	ST_R_GET_INF *stGetInfo;

	App_Init();
	
	if (gSys.com_ok == NG) {
		printf("ComPort Open Fail\n");
		exit(0);
	}
	
	// 製品情報取得
	GetProductInfo();
	while(1) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			CommRcvBuff[0]=0; 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
				ROS_INFO_STREAM("Version : " << stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
				ROS_INFO_STREAM("SerialNo : " << stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
				ROS_INFO_STREAM("Type : " << stGetInfo->scPName);
				EndF = 1;
			}
			
		}
		if ( EndF==1 ) break;
	}
	
	usleep(1000);

	// ROS Config
	ros::init(argc, argv, "leptorino_node");
	ros::NodeHandle node_handler;
	ros::Publisher pub = node_handler.advertise<std_msgs::Float32MultiArray>("sensor_data", 1);
	ros::Rate timer(100);

	// Calibration data
	std_msgs::Float32MultiArray calib_data;
	calib_data.data = {0,0,0,0,0,0};

	// Plot configration
	Gpop::Series plot;
	plot.limit_max_number(500);
	plot.set_x_label("t");
	plot.set_y_label("N");

	// Log configuration
	// 連続送信開始
	SerialStart();
	EndF = 0;

	while(ros::ok()) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			memset(CommRcvBuff,0,sizeof(CommRcvBuff)); 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				cnt++;
				
				const int calib_num = 10;
				if (cnt <= calib_num) {	//calibration
					stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
					calib_data.data.at(0) += (double)stForce->ssForce[0];
					calib_data.data.at(1) += (double)stForce->ssForce[1];
					calib_data.data.at(2) += (double)stForce->ssForce[2];
					calib_data.data.at(3) += (double)stForce->ssForce[3];
					calib_data.data.at(4) += (double)stForce->ssForce[4];
					calib_data.data.at(5) += (double)stForce->ssForce[5];
					ROS_INFO_STREAM("Calibration now : " << "current value : " << stForce->ssForce[0] << "\t" << "current bias: " << calib_data.data.at(0) / (double)calib_num);
					ros::Duration(0.1).sleep();
				}
				else {	//publish
					stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
					std_msgs::Float32MultiArray sensor_data;
					sensor_data.data = {0,0,0,0,0,0};
					sensor_data.data.at(0) = stForce->ssForce[0] - calib_data.data.at(0)/(double)calib_num;
					sensor_data.data.at(1) = stForce->ssForce[1] - calib_data.data.at(1)/(double)calib_num;
					sensor_data.data.at(2) = stForce->ssForce[2] - calib_data.data.at(2)/(double)calib_num;
					sensor_data.data.at(3) = stForce->ssForce[3] - calib_data.data.at(3)/(double)calib_num;
					sensor_data.data.at(4) = stForce->ssForce[4] - calib_data.data.at(4)/(double)calib_num;
					sensor_data.data.at(5) = stForce->ssForce[5] - calib_data.data.at(5)/(double)calib_num;
					pub.publish(sensor_data);
					//plot.plot(leptorino::cliping(sensor_data.data.at(0)));
					plot.plot(sensor_data.data.at(0));
					plot.pause();
				}

				// 連続送信停止
				//if (cnt == 10000) { SerialStop();}

				stCmdHead = (ST_RES_HEAD *)CommRcvBuff;
				if (stCmdHead->ucCmd == CMD_DATA_STOP) {
					printf("Receive Stop Response:");
					l = stCmdHead->ucLen;
					for ( i=0; i<l; i++) {
						printf("%02x ", CommRcvBuff[i]);
					}
					printf("\n");
					EndF = 1;
				}
			}
			timer.sleep();
		}
		if ( EndF==1 ) break;
	}
	App_Close();
	return 0;
}

// ----------------------------------------------------------------------------------
//	アプリケーション初期化
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Init(void)
{
	int rt;
	
	//Commポート初期化
	gSys.com_ok = NG;
	rt = Comm_Open("/dev/ttyACM0");
	if ( rt==OK ) {
		Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
		gSys.com_ok = OK;
	}

}

// ----------------------------------------------------------------------------------
//	アプリケーション終了処理
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Close(void)
{
	printf("Application Close\n");
	
	if ( gSys.com_ok == OK) {
		Comm_Close();
	}
}

/*********************************************************************************
* Function Name  : HST_SendResp
* Description    : データを整形して送信する
* Input          : pucInput 送信データ
*                : 送信データサイズ
* Output         : 
* Return         : 
*********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
	USHORT usCnt;
	UCHAR ucWork;
	UCHAR ucBCC = 0;
	UCHAR *pucWrite = &CommSendBuff[0];
	USHORT usRealSize;
	
	// データ整形 
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_STX;					// STX 
	pucWrite++;
	usRealSize =2;
	
	for (usCnt = 0; usCnt < usSize; usCnt++) {
		ucWork = pucInput[usCnt];
		if (ucWork == CHR_DLE) {			// データが0x10ならば0x10を付加 
			*pucWrite = CHR_DLE;			// DLE付加 
			pucWrite++;						// 書き込み先 
			usRealSize++;					// 実サイズ
			// BCCは計算しない!
		}
		*pucWrite = ucWork;					// データ 
		ucBCC ^= ucWork;					// BCC 
		pucWrite++;							// 書き込み先 
		usRealSize++;						// 実サイズ 
	}
	
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_ETX;					// ETX 
	ucBCC ^= CHR_ETX;						// BCC計算 
	pucWrite++;
	*pucWrite = ucBCC;						// BCC付加 
	usRealSize += 3;
	
	Comm_SendData(&CommSendBuff[0], usRealSize);
	
	return OK;
}

void GetProductInfo(void)
{
	USHORT len;
	
	printf("Get SensorInfo\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_GET_INF;				// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStart(void)
{
	USHORT len;
	
	printf("Start\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_START;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStop(void)
{
	USHORT len;
	
	printf("Stop\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_STOP;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

