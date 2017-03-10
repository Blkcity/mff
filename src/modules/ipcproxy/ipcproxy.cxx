#include "ipcproxy.hpp"

IpcProxy::IpcProxy(const char* keyname, ipc_type_t type) :
	mIpcHandle(NULL)
{
	mIpcType = type;
	if (type == IPC_TYPE_RECEIVER) 
	{
		mIpcHandle = ipcCreate(0, (char*)keyname, 0666, 512);
		if (mIpcHandle == NULL) 
		{
			printf("ipc failed: can't create ipc queue: %s\n", keyname);
			exit(1);
		}
	}
	else if (type == IPC_TYPE_SENDER) 
	{
		mIpcHandle = ipcOpen(0, (char*)keyname,512);
		if (mIpcHandle == NULL) 
		{
			printf("IpcProxy failed: can't open ipc queue: %s\n", keyname);
			return;
		}
	}
}

IpcProxy::~IpcProxy()
{
	if (mIpcHandle != NULL) 
	{
		if (mIpcType == IPC_TYPE_RECEIVER) 
			ipcDelete(mIpcHandle);
		else if (mIpcType == IPC_TYPE_SENDER)
			ipcClose(mIpcHandle);
		mIpcHandle = NULL;
	}
}

int IpcProxy::sendMsg(void* buf, size_t len, int async)
{
	int rst = -1;
	if (mIpcHandle == NULL) 
	{
		mFailedCnt++;
		return rst;
	}
	rst = ipcSend(mIpcHandle, buf, len, async);
	if (rst < 0)
		mFailedCnt++;
	else
		mFailedCnt = 0;
	
	return rst;
}

int IpcProxy::recvMsg(void* buf)
{
	int rst = -1;
	if (mIpcHandle == NULL) 
	{
		mFailedCnt++;
		return rst;
	}
	rst = ipcReceive(mIpcHandle, buf);
	if (rst < 0)
		mFailedCnt++;
	else
		mFailedCnt = 0;

	return rst;
}

int IpcProxy::emptyMsg()
{
	int rst = -1;
	if (mIpcHandle == NULL)
		return rst;

	rst = ipcEmpty(mIpcHandle);
	return rst;
}


