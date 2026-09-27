#include "ID_CAN.h"
#include <Arduino.h>

#define OFFSET_CARTE_MPP_AVANT 0x000
#define OFFSET_CARTE_MPP_ARRIERE 0x010
#define CONFIG_ADDR 0x0800F800  // dernière page Flash

uint32_t getRoleFromFlash();
extern uint32_t CARD_ROLE;
extern uint16_t offset_can_id;

struct USED_ID {
    uint16_t BOOT_CARTE_MPP;
    uint16_t HERKULEX_AIMANT_COTE;
    uint16_t HERKULEX_AIMANT_CENTRE;
    uint16_t HERKULEX_PIVOT_COTE; 
    uint16_t CMD_MPP;
    uint16_t HERKULEX_PIVOT_POMPE;
    uint16_t LACHER;
    uint16_t CONSTRUIRE_2ETAGE;
    uint16_t CONSTRUIRE_PREPARER; 
    uint16_t CONSTRUIRE_TERMINEE;
    uint16_t RESTART_STM; 
};

extern USED_ID USED_CAN_ID; // déclaration externe




