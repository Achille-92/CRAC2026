#ifndef _STEPPER_H
#define _STEPPER_H

#define ANGLE_PAS_PAR 0.9
#define PAS_PAR_TOUR 400    
#define COURANT_PAR_PHASE 0.9
#define COUPLE_MAINTIEN 3.6 //3,6 kg.cm == 36 Ncm

#define PAS_COMPLET        0b000  // M2=0, M1=0, M0=0
#define DEMI_PAS          0b001  // M2=0, M1=0, M0=1
#define QUART_DE_PAS      0b010  // M2=0, M1=1, M0=0
#define HUITIEME_DE_PAS   0b011  // M2=0, M1=1, M0=1
#define SEIZIEME_DE_PAS   0b100  // M2=1, M1=0, M0=0
#define TRENTEDEUXIEME_DE_PAS 0b101  // M2=1, M1=0, M0=1
#define SOIXANTEQUATRIEME_DE_PAS 0b110  // M2=1, M1=1, M0=0
#define CENTVINGTHUITIEME_DE_PAS 0b111  // M2=1, M1=1, M0=1
#define SENS_HAUT 0
#define SENS_BAS 1

//* Fonction pour initialiser le moteur pas-à-pas
void initStepper();
void haut (int pas);
void bas (int pas);

#endif
