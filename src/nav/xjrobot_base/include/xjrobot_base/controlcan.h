#ifndef XJROBOT_BASE__CONTROLCAN_H_
#define XJROBOT_BASE__CONTROLCAN_H_

#define VCI_USBCAN1 3
#define VCI_USBCAN2 4
#define VCI_USBCAN2A 4

#define VCI_USBCAN_E_U 20
#define VCI_USBCAN_2E_U 21

#define STATUS_OK 1
#define STATUS_ERR 0

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void *
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void *
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

typedef struct _VCI_BOARD_INFO
{
  USHORT hw_Version;
  USHORT fw_Version;
  USHORT dr_Version;
  USHORT in_Version;
  USHORT irq_Num;
  BYTE can_Num;
  CHAR str_Serial_Num[20];
  CHAR str_hw_Type[40];
  USHORT Reserved[4];
} VCI_BOARD_INFO, *PVCI_BOARD_INFO;

typedef struct _VCI_CAN_OBJ
{
  UINT ID;
  UINT TimeStamp;
  BYTE TimeFlag;
  BYTE SendType;
  BYTE RemoteFlag;
  BYTE ExternFlag;
  BYTE DataLen;
  BYTE Data[8];
  BYTE Reserved[3];
} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

typedef struct _INIT_CONFIG
{
  DWORD AccCode;
  DWORD AccMask;
  DWORD Reserved;
  UCHAR Filter;
  UCHAR Timing0;
  UCHAR Timing1;
  UCHAR Mode;
} VCI_INIT_CONFIG, *PVCI_INIT_CONFIG;

typedef struct _VCI_FILTER_RECORD
{
  DWORD ExtFrame;
  DWORD Start;
  DWORD End;
} VCI_FILTER_RECORD, *PVCI_FILTER_RECORD;

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved);
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd);
EXTERN_C DWORD VCI_InitCAN(
  DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);
EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType, DWORD DeviceInd, PVCI_BOARD_INFO pInfo);
EXTERN_C DWORD VCI_SetReference(
  DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);
EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C ULONG VCI_Transmit(
  DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, UINT Len);
EXTERN_C ULONG VCI_Receive(
  DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, UINT Len,
  INT WaitTime);
EXTERN_C DWORD VCI_UsbDeviceReset(DWORD DevType, DWORD DevIndex, DWORD Reserved);
EXTERN_C DWORD VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

#endif
