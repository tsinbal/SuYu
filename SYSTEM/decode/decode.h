#ifndef __DECODE_H
#define __DECODE_H	 
#include "sys.h"
#include "config.h"
/*{
 cmd:'10002',
 v: '0001',
 body: {
     volume: '2',    // (0-100)
     sound: '2',     // (0-3)
     calibrate: '4', // (0-1)
     gpsx: '',       // 127,10   
     gpst: '',       // (0-2)   
     speed: '',      // (0-30)  
     scram: '',      // (0-1)   
     gears: '',      // (0-5)   
 }
}

*/


#define BODY_APPLEN_CODE "applength"
#define BODY_MD5_CODE "md5"
#define BODY_CRC32_CODE "crc32"

#define BODY_VOLUME_CODE "volume"
#define BODY_SOUND_CODE "sound"
#define BODY_CALIBRATE_CODE "calibrate"
#define BODY_GEARS_CODE "gears"

#define MAX_BODY_LENGTH (USART5_REC_LEN-30)




typedef struct BodyKeyValue {
	
	char *key;			
	char *value;				
} BodyKeyValue;
void Json_Decode(int *commandid,int *version,u8 * body);
void Body_Decode(u8 paraNum,u8	*body, BodyKeyValue *keyvalue,...);
#endif


