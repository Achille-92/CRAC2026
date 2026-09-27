#include <PID.h>
#include <Arduino.h>
#include <utility.h>
PID::PID(float kp, float ki, float kd, float max_integral, float max_sortie)
{
    Kp = kp;
    Kd = kd;
    Ki = ki;
    Max_integral = max_integral;
    Max_sortie = max_sortie;
}
float PID::calcul(float erreur)
{
    float sortie_avant_contrainte = Kp * erreur + Ki * integral + Kd * (erreur - erreur_pre);
    
    // Anti-windup : n'intègre pas si on est déjà saturé
    bool sature = (sortie_avant_contrainte > Max_sortie || sortie_avant_contrainte < -Max_sortie);
    bool meme_signe = (erreur * integral > 0); // erreur aggrave la saturation
    
    if (!(sature && meme_signe))
        integral += erreur;
    
    integral = constrain(integral, -Max_integral, Max_integral);
    float sortie = Kp * erreur + Ki * integral + Kd * (erreur - erreur_pre);
    sortie = constrain(sortie, -Max_sortie, Max_sortie);
    erreur_pre = erreur;
    return sortie;
}
void PID::reset(void)
{
    integral = 0;
}
void PID::setKi(float ki)
{
    Ki = ki;
}
void PID::setKp(float kp)
{
    Kp = kp;
}
void PID::setKd(float kd)
{
    Kd = kd;
}
void PID::setMaxIntegral(float max_integral)
{
    Max_integral = max_integral;
}
void PID::setMaxSortie(float max_sortie)
{
    Max_sortie = max_sortie;
}