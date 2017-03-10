#include "ipcinterface.h"

ipcHandle* ipcSendHandle = NULL;
ipcHandle* ipcReceiveHandle = NULL;

ipcHandle* ipcCreate(int ipcType,char *keyName,int mode,int len)
{
	if (ipcType != MsgType)
	{
	    printf("func(%s),ipcType:%d is not equ MsgType:%d\n",__func__,ipcType,MsgType);
	    return NULL;
	}

	if(len > MSGQUEMAXCAP) 
	{
	    printf("message queue length more than maxMsgQueCap\n");
	    return NULL;
	} 

	ipcReceiveHandle = (ipcHandle*) malloc(sizeof(ipcHandle));
	memset(ipcReceiveHandle,0,sizeof(ipcHandle));

	memcpy(ipcReceiveHandle->keyName,keyName,strlen(keyName));

	ipcReceiveHandle->mq_stat.mq_maxmsg = MSGMAXSIZE;
	ipcReceiveHandle->mq_stat.mq_msgsize = len;
	ipcReceiveHandle->mq_stat.mq_flags = 0;

	ipcReceiveHandle->mqdes = mq_open(keyName,O_RDONLY|O_CREAT,0666,&ipcReceiveHandle->mq_stat);

	if(ipcReceiveHandle->mqdes == (mqd_t)-1) 
	{
	    printf("create msgq failed\n");
	    free(ipcReceiveHandle);
	    return NULL;
	}
	printf("%s, create ok, %s\n", __func__, keyName);
	ipcReceiveHandle->ipcType = ipcType;

	return ipcReceiveHandle;
}

ipcHandle* ipcOpen(int ipcType, char *keyName, int len)
{
	if (ipcType != MsgType)
	{
	    printf("func(%s),ipcType:%d is not equ MsgType:%d\n",__func__,ipcType,MsgType);
	    return NULL;
	}

	if(len > MSGQUEMAXCAP) 
	{
	    printf("message queue length more than maxMsgQueCap\n");
	    return NULL;
	} 

	ipcSendHandle = (ipcHandle*) malloc(sizeof(ipcHandle));
	memset(ipcSendHandle, 0, sizeof(ipcHandle));

	ipcSendHandle->mq_stat.mq_maxmsg = MSGMAXSIZE;
	ipcSendHandle->mq_stat.mq_msgsize = len;
	ipcSendHandle->mq_stat.mq_flags = 0;

	ipcSendHandle->mqdes = mq_open(keyName, O_WRONLY, 0666,&ipcSendHandle->mq_stat);
	if(ipcSendHandle->mqdes == (mqd_t)-1)
	{
	    printf("%s, open fail,%s\n",__func__,keyName);
	    free(ipcSendHandle);
	    return NULL;
	}

	printf("%s, open ok, %s\n", __func__, keyName);
	ipcSendHandle->ipcType = ipcType;

	return ipcSendHandle;
}

int ipcReceive(ipcHandle* ReceiveHandle, void* buf)
{
	int ret = 0;
	struct mesg msgs;

	if(ReceiveHandle == NULL)
	    return -1;
	if(ReceiveHandle->ipcType != MsgType)
	    return -1;

	memset(msgs.mdata, 0, (sizeof(char)*MSGMAXSIZE));
	ret = mq_receive(ReceiveHandle->mqdes, msgs.mdata, MSGMAXSIZE, NULL);
	if(ret < 0)
	    return -1;

	memcpy(buf, (void*)(msgs.mdata), ret);

	return ret;
}

int ipcSend(ipcHandle* SendHandle, void* buf, size_t len, int async)
{
	struct mesg msgs;
	//printf("ipcSend 11111\n");
	
	if(ipcSendHandle == NULL)
	    return -1;
	if(SendHandle->ipcType != MsgType)
	    return -1;
	if(len <= 0)
	    return -1;

	msgs.mtype = 1;//attention
	if(len >= MSGMAXSIZE)
	    return -1;
	//printf("ipcSend ipcId = %d\n", ipcSendHandle->ipcId);
	
	memset(msgs.mdata, 0, sizeof(char)*MSGMAXSIZE);
	memcpy(msgs.mdata, buf, len);

	//0 : wait to send;IPC_NOWAIT :No wait
	if(mq_send(SendHandle->mqdes, msgs.mdata, len, 0) < 0)
	    return -1;

	return 0;
}

int ipcClose(ipcHandle* SendHandle)
{
	if(SendHandle == NULL)
		return -1;

	if(SendHandle->ipcType != MsgType)
		return -1;
	mq_close(SendHandle->mqdes );
	free(SendHandle);
	SendHandle = NULL;

	return 0;
}

int ipcDelete(ipcHandle* ReceiveHandle)
{
	if(ReceiveHandle == NULL)
		return -1;
	if(ReceiveHandle->ipcType != MsgType)
	    return -1;

	ipcClose(ReceiveHandle);
	mq_unlink((FAR const char *)ReceiveHandle->keyName);

	free(ipcReceiveHandle);
	ipcReceiveHandle = NULL;

	return 0;
}

int ipcEmpty(ipcHandle* ReceiveHandle)
{
    return 0;
}
