#include "../ipcinterface.h"

__EXPORT int receive_main(int argc, char *argv[]);

int receive_main(int argc, char *argv[])
{
    int running = 1;
    char buffer[64];

    ipcHandle  *pHandle;
    pHandle =ipcCreate(0,"testipc",0666,64);
    if(pHandle == NULL)
    {
	printf("ipcreceive open fail!\n");
	return -1;
    }

    while(running)
    {
	memset(buffer,0,64);

        if(ipcReceive(pHandle, (void*)buffer) == -1)
        {
	    printf("ipcReceive fail!\n");
	    return -1;
        }
        printf("You wrote: %s\n",(char*)buffer);

        if(strncmp((char*)buffer, "end", 3) == 0)
            running = 0;
    }

    if(ipcDelete(pHandle))
    {
	printf("ipcdelete fail!\n");
	return -1;
    }
    return 0;
}
