#include "STM32_CAN.h"
#include <CAN.h>
// #include "CONFIG_CARTE.h"

STM32_CAN Can( CAN1, DEF );  //Use PA11/12 pins for CAN1.
//STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.
//STM32_CAN Can( CAN1, ALT_2 );  //Use PD0/1 pins for CAN1.
//STM32_CAN Can( CAN2, DEF );  //Use PB12/13 pins for CAN2.
//STM32_CAN Can( CAN2, ALT );  //Use PB5/6 pins for CAN2
//STM32_CAN Can( CAN3, DEF );  //Use PA8/15 pins for CAN3.
//STM32_CAN Can( CAN3, ALT );  //Use PB3/4 pins for CAN3

static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

void setup_can(){

    Can.begin();
    //Can.setBaudRate(250000);  //250KBPS
    Can.setBaudRate(500000);  //500KBPS
    //Can.setBaudRate(1000000);  //1000KBPS
  
}

void sendCANMessage(int id, int data0, int data1, int data2, int data3, int data4, int data5, int data6,int data7)
{
    // Exemple : Envoi d'un message CAN
    CAN_TX_msg.id = id; // ID CAN
    CAN_TX_msg.len = 8; // DLC : Nombre d'octets dans le message
    CAN_TX_msg.buf[0] = data0;      // Données a envoyés
    CAN_TX_msg.buf[1] = data1;
    CAN_TX_msg.buf[2] = data2;
    CAN_TX_msg.buf[3] = data3;
    CAN_TX_msg.buf[4] = data4;
    CAN_TX_msg.buf[5] = data5;
    CAN_TX_msg.buf[6] = data6;
    CAN_TX_msg.buf[7] = data7;

    Can.write(CAN_TX_msg);
}

// id : l'id de la trame CAN reçu  
// adresse_tableau_data : adresse du tableau dans laquelle on stocke les datas
// Retourne 1 quand le msg est pour nous, 0 sinon ou si pas de msg
// range les valeurs dans les variables d'entrées
char receiveCANMessage(int *id, char *adresse_tableau_data){ 
    if(Can.read(CAN_RX_msg)){
        if(msg_for_me(CAN_RX_msg.id)){
            *id = CAN_RX_msg.id;
            for(int i = 0; i < 8; i++){
                *(adresse_tableau_data+i) = CAN_RX_msg.buf[i]; // met les datas dans le tableau
            }
            return 1; // retourne 1 pour dire que un msg est reçu et que c'est un id que l'on veut traiter
        }
        return 0; // pas id que l'on traite
    }
    else{
        return 0;
    } // pas de msg reçu
}

// filtre le message, retourne 1 si msg pour nous
bool msg_for_me(int id_msg_rx){
    Serial.println(id_msg_rx,HEX);
    // msg pour nous, ceux de la carte avant si configuré en carte avant, carte arriere sinon
    if(id_msg_rx == USED_CAN_ID.HERKULEX_AIMANT_CENTRE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.HERKULEX_AIMANT_COTE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.HERKULEX_PIVOT_COTE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.CMD_MPP){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.HERKULEX_PIVOT_POMPE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.LACHER){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.CONSTRUIRE_PREPARER){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.CONSTRUIRE_2ETAGE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.CONSTRUIRE_TERMINEE){
        return 1;
    }
    if(id_msg_rx == USED_CAN_ID.RESTART_STM){
        return 1;
    }

    // msg pas pour nous
    return 0; 
}