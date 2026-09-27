#include <Vitesse.h>
#include <Arduino.h>
#include <PID.h>
#include <utility.h>

vitesse::vitesse() : droit_pid(KPdroit, KIdroit, KdVitesse, (300.0f / KIdroit), 300.0f),
                     gauche_pid(KPgauche, KIgauche, KdVitesse, (300.0f / KIgauche), 300.0f)
{
}
float vitesse::droit(float consigne, float sortie, int type, float dist)
{
    if (dist != 0)
        consigne = constrain(consigne, -dist / 200.0, dist / 200.0);
    if (type == 2)
    {
        consigne = constrain(consigne, -0.5, 0.50);
        droit_pid.setKd(100);
    }

    else
    {
        consigne = constrain(consigne, -1.0, 1.0);
        droit_pid.setKd(KdVitesse);
    }
    float erreurd = (consigne - sortie);
    if (fabs(consigne) < 0.05)
        droit_pid.reset();
    float PWMD = droit_pid.calcul(erreurd);
    return PWMD;
}
float vitesse::gauche(float consigne, float sortie, int type, float dist)
{
    if (dist != 0)
        consigne = constrain(consigne, -dist / 200.0, dist / 200.0);
    if (type == 2)
    {
        consigne = constrain(consigne, -0.5, 0.50);
        gauche_pid.setKd(100);
    }

    else
    {
        consigne = constrain(consigne, -1.0, 1.0);
        gauche_pid.setKd(KdVitesse);
    }
    float erreurg = (consigne - sortie);
    if (fabs(consigne) < 0.05)
        gauche_pid.reset();
    float PWMG = gauche_pid.calcul(erreurg);
    return PWMG;
}
void vitesse::reset(void)
{
    droit_pid.reset();
    gauche_pid.reset();
}
