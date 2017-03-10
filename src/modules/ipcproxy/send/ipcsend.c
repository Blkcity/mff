#include "../ipcinterface.h"

__EXPORT int send_main(int argc, char *argv[]);

int send_main(int argc, char *argv[])
{
    int running = 1;
    int ch,i;

    char buffer[64];

    ipcHandle *pHandle;

    pHandle = ipcOpen(0,"testipc",64);
    if(pHandle == NULL)
    {
	printf("ipc open fail!\n");
	return -1;
    }
    while(running)
    {
	memset(buffer,0,64);
	ch = 0;
	i = 0;
	while(1)
	{
	    ch = getchar();
	    if (ch == '\n' || ch == '\r')
		break;
	    else
		buffer[i++] = (char)ch;
	    if(i>63)
		break;
	    printf("%c",ch);
	}
	printf("\r\n");
	if(i>0)
	{
	    if(ipcSend(pHandle,(void*)buffer, strlen(buffer),0) == -1)
	    {
		printf("ipcsend fail!\n");
		return -1;
	    }	    
	    if(strncmp(buffer, "end", 3) == 0)
		running = 0;
	}
    }
    return 1;
}
