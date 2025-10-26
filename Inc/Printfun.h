#ifndef PRINTFUNC_H
#define PRINTFUNC_H


uint16_t  PrintMsg(uint16_t cnt);
uint16_t EncodeCommandStatus(STATUSES1_E Status);
uint16_t  EncodeUUId(uint32_t UUid,uint16_t Ver);
uint32_t u32Alder( uint8_t *pu8Data, uint16_t u32Len,uint32_t *a ,uint32_t *b);


#endif