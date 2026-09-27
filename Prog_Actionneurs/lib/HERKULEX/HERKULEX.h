#ifndef _HERKULEX_H
#define _HERKULEX_H
#include <HardwareSerial.h>
#include <HerkulexServo.h>

#define SERVO_AIMANT_CENTRE  0x04
#define SERVO_AIMANT_GAUCHE  0x02 
#define SERVO_AIMANT_DROIT  0x03
#define SERVO_PIVOT_GAUCHE 0x01
#define SERVO_PIVOT_DROIT 0x05
#define SERVO_ROTATION 0x06
#define SERVO_PIVOT_PINCE 0x07

#define RETIRER 0
#define ATTRAPER 1
#define CENTRE 0 // canette cote ramené au centre
#define COTE 1 // canette cote ramené au coté
#define ECARTER 2 // cannette cote écarté pour monter la planche 
#define RETRACTER 0
#define DEPLOYER 1
#define AVANT_CONSTRUCTION 2

#define ANGLE_PINCE_ATTRAPER 130 
#define ANGLE_PINCE_LACHER 0
#define ANGLE_PIVOT_PINCE_AVANT_CONSTRUCTION 80
#define ANGLE_PIVOT_COTE_ECARTER 25 // + pour le droit, - gauche
#define ANGLE_PIVOT_COTE_ATTRAPER -50
#define ANGLE_PIVOT_COTE_CENTRE -113




void change_id(uint8_t id, HerkulexServo old_, HerkulexServo new_);
void init_serial_1_for_herkulex();
void test_herkulex();
void test_connexion();
int detect_id(bool activate);
void rotation_moteur(void);
void aimant_cote_ecarter(void);
void aimant_cote_attraper(void);
void cmd_aimant_centre(bool mouvement);
void cmd_aimant_cote(char mouvement);
void cmd_pivot_pompe(char mouvement);
void cmd_pince(bool mouvement);
void display_servo_position(void);
int16_t get_servo_pos(HerkulexServo servo);
void restart_all_servo(void);
void get_all_servo_pos(
    short *pos_servo_pivot_gauche,
    short *pos_servo_pivot_droit,
    short *pos_servo_aimant_droit,
    short *pos_servo_aimant_gauche,
    short *pos_servo_aimant_centre,
    short *pos_servo_pince,
    short *pos_servo_pivot_pince);



#endif

// Définition des broches utilisées pour la communication série avec les servos Herkulex
#define PIN_SW_RX PB7 // Broche utilisée pour la réception (RX)
#define PIN_SW_TX PB6 // Broche utilisée pour la transmission (TX)


/*
    J'ai décalé les créations d'objet et define ici pour y avoir accès partout
    Comme ça je peux aussi m'en servir dans le main au lieu de recréer des objets
    et créer un problème avec 2 fois le même objet mais qui n'a pas le même nom
*/


// Création d'un objet servo avec l'adresse de diffusion (HERKULEX_BROADCAST_ID signifie tous les servos connectés)
