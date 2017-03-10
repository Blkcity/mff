#ifndef _IpcProxy_hpp
#define _IpcProxy_hpp

extern "C" {
#include "ipcinterface.h"
}

typedef enum {
	IPC_TYPE_RECEIVER = 0x01,
	IPC_TYPE_SENDER = 0x02
}ipc_type_t;

class IpcProxy
{
private:
	ipc_type_t mIpcType;
	ipcHandle* mIpcHandle;
	int32_t mFailedCnt;

public:
	IpcProxy(const char* keyname, ipc_type_t type);
	~IpcProxy();
	int sendMsg(void* buf, size_t len, int async); // send msg through ipc
	int recvMsg(void* buf); // recv msg through ipc
	int emptyMsg();
	int getFailedCnt() {return mFailedCnt;}
};

#endif // _IpcProxy_hpp
