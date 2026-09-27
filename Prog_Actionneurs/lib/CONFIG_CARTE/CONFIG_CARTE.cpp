#include "CONFIG_CARTE.h"

uint32_t getRoleFromFlash() {
    return *(volatile uint8_t*)CONFIG_ADDR;
  }

uint32_t CARD_ROLE = getRoleFromFlash();
uint16_t offset_can_id = (CARD_ROLE) ? OFFSET_CARTE_MPP_AVANT : OFFSET_CARTE_MPP_ARRIERE;

USED_ID USED_CAN_ID = {
    .BOOT_CARTE_MPP = (uint16_t)(BOOT_CARTE_MPP_AVANT + offset_can_id),
    .HERKULEX_AIMANT_COTE = (uint16_t)(HERKULEX_AVANT_AIMANT_COTE + offset_can_id),
    .HERKULEX_AIMANT_CENTRE = (uint16_t)(HERKULEX_AVANT_AIMANT_CENTRE + offset_can_id),
    .HERKULEX_PIVOT_COTE = (uint16_t)(HERKULEX_AVANT_PIVOT_COTE + offset_can_id),
    .CMD_MPP = (uint16_t)(CMD_MPP_AVANT + offset_can_id),
    .HERKULEX_PIVOT_POMPE = (uint16_t)(HERKULEX_AVANT_PIVOT_POMPE + offset_can_id),
    .LACHER = (uint16_t)(LACHER_AVANT + offset_can_id),
    .CONSTRUIRE_2ETAGE = (uint16_t)(CONSTRUIRE_AVANT_2ETAGE + offset_can_id),
    .CONSTRUIRE_PREPARER = (uint16_t)(CONSTRUIRE_AVANT_PREPARER + offset_can_id),
    .CONSTRUIRE_TERMINEE = (uint16_t)(CONSTRUIRE_AVANT_TERMINEE + offset_can_id),
    .RESTART_STM = (uint16_t)(RESTART_AVANT + offset_can_id),
};