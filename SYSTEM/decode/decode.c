#include "decode.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cJSON.h"
#include <stdlib.h>
#include <stdio.h>
#include "stdarg.h"
void Json_Decode(int *commandid,int *version,u8 * body)
{

    cJSON * root = NULL;
    cJSON * item = NULL;//cjson
    char *temp=NULL;
    root = cJSON_Parse(USART5_RX_BUF);
    //printf(USART5_RX_BUF);
    if (!root) {
        printf("Error DeCode Json Data!\r\n");
        return;
    }
    if(commandid!=NULL) {
        item = cJSON_GetObjectItem(root,COMMAND_CODE);//
        *commandid=atoi(item->valuestring);
    }
    if(version!=NULL) {
        item = cJSON_GetObjectItem(root,VERSION_CODE);//
        *version=atoi(item->valuestring);
    }
    if(body!=NULL) {
        item = cJSON_GetObjectItem(root,BODY_CODE);//
        temp=cJSON_PrintUnformatted(item);
        memcpy(body,temp,strlen(temp)+1);
    }
    cJSON_Delete(root);
    vPortFree(temp);
}

void Body_Decode(u8 paraNum,u8	*body, BodyKeyValue *keyvalue,...)
{

    cJSON * root = NULL;
    cJSON * item = NULL;//cjson
    u8 i=0;
    va_list argptr;
    va_start(argptr, keyvalue);
    root = cJSON_Parse(body);
    if (!root) {
        printf("Error DeCode Body Data!\r\n");
        return;
    }
    for(; i<paraNum; i++) {
        if(keyvalue!=NULL&&keyvalue->key!=NULL) {
            item = cJSON_GetObjectItem(root,keyvalue->key);

            if(((item->type)&255)==cJSON_String) {
                keyvalue->value=pvPortMalloc(strlen(item->valuestring)+1);
                memcpy(keyvalue->value,item->valuestring,strlen(item->valuestring)+1);
            }
        }
        keyvalue=va_arg(argptr,BodyKeyValue *);
    }


    cJSON_Delete(root);


}
