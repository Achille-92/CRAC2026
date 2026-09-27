#ifndef _CAN_H
#define _CAN_H
#include <ID_CAN.h>
#include "CONFIG_CARTE.h"

void setup_can();
void sendCANMessage(int id, int data0, int data1, int data2, int data3, int data4, int data5, int data6,int data7);
char receiveCANMessage(int *id, char *adresse_tableau_data); // regarde si un message est arrivé, si oui et si l'id est pour nous range l'id à l'aide de l'id
bool msg_for_me(int id_msg_rx); // retourne 1 si msg pour nous, 0 sinon

#endif
