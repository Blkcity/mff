#ifndef _IPCINTERFACE_H_
#define _IPCINTERFACE_H_
/*
 * Ipc interface for process communication
 * Copyright (c) 2016, Xiaobo He <hexiaobo@fimi>
 *
 * This software may be distributed under the terms of fimi.
 */
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <mqueue.h>
#include <sched.h>
#include <errno.h>
#include <fmx10.h>

#define MsgType 0
#define ShmType 1
#define MSGQUEMAXCAP  1024*256
#define MSGQUEMAXNUM  64
#define MSGMAXSIZE 64

typedef struct IPCHandle 
{
    FAR struct mq_attr mq_stat;
    mqd_t mqdes;
    char keyName[64];
    int ipcType;
}ipcHandle;

struct mesg {
    long int mtype;
    char  mdata[MSGMAXSIZE];
};

 typedef enum
 {
     MSG_TYPE = 0x00,
     SHM_TYPE = 0x01,
 }IPC_TYPE;

/****************************************
function :
    Create ipc receive handle 
parameters:
    ipcType:
        0  MSG_TYPE
        (1  SHM_TYPE //this type not implemented)
    keyName:
        character string
    mode:
      IPC operating rights, similar to the file operation 
      Eg.0666
    len:
      Message queue size //this funtion not implemented,the vale
      will use defaule 163800
reutrn :
    SUCCESS  return ipcHandle Pointer;
    Fail  return NULL;
*******************************************/
ipcHandle* ipcCreate(int ipcType,char* keyName,int mode,int len);

/****************************************
function :
    Open ipc send handle
parameters:
    ipcType: 
        0  MSG_TYPE
    keyName:
        character string

reutrn :
    SUCCESS  return ipcHandle Pointer;
    Fail  return NULL;
*******************************************/
ipcHandle* ipcOpen(int ipcType,char* keyName, int len);

/****************************************
function :
    ipc reveive mesage
parameters:
    ipcReceiveHandle:
        ipcHandle pointer
    buf:
        void type pointer
reutrn :
    SUCCESS return receive msg length;
    Fail  return -1;
*******************************************/
int ipcReceive(ipcHandle* ReceiveHandle, void* buf);

/****************************************
function :
    ipc send mesage
parameters:
    ipcSendHandle:
        ipcHandle pointer
    buf:
        void type pointer
    len:
        size_t type
    async:
        0 : wait to send;
        IPC_NOWAIT :No wait
reutrn :
    SUCCESS return 0;
    Fail  return -1;
*******************************************/
int ipcSend(ipcHandle* SendHandle, void* buf, size_t len, int async);

/****************************************
function :
    delete ipc receive handle
parameters:
    ipcReceiveHandle:  ipcHandle pointer
reutrn :
    SUCCESS return 0;
    Fail  return -1;
*******************************************/
int ipcDelete(ipcHandle* ReceiveHandle);

/****************************************
function :
    close send ipc handle
parameters:
    ipcSendHandle:  ipcHandle pointer
reutrn :
    SUCCESS return 0;
    Fail  return -1;
*******************************************/
int ipcClose(ipcHandle* SendHandle);

/****************************************
function :
    close ipc receive handle
parameters:
    ipcReceHandle:  ipcHandle pointer
reutrn :
    SUCCESS return 0;
    Fail  return -1;
*******************************************/
int ipcEmpty(ipcHandle* ReceiveHandle);

#endif /* _IPCINTERFACE_H_ */

