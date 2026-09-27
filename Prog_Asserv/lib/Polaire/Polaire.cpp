#include <Polaire.h>
#include <Arduino.h>
#include <PID.h>
#include <utility.h>

Polaire::Polaire() : lineaire(KPlin, KIlin, KDlin, 0.2f / KIlin, 1.0f),
                     angulaire(KPang, KIang, KDang, 0.2f / KIang, 0.8f)
{
}

float Polaire::avant(float consigne, float sortie, float teta_act, float teta, int type)
{
    if (type == 0)
    {

        lineaire.setKd(0.05f);
        lineaire.setKp(0.015f);
    }
    else
    {

        lineaire.setKd(KDlin);
        lineaire.setKp(KPlin);
    }
    float diff_tetaa = teta - teta_act;
    float VIT;
    diff_tetaa = limit_angle(diff_tetaa);
    float erreura = consigne - sortie;
    if (consigne < 20.0 || consigne > 30.0)
        lineaire.reset();
    if (type == 2)
    {
        VIT = pow(cos(diff_tetaa), 55) * lineaire.calcul(erreura);
    }
    else
    {

        if (consigne > 40.0f)
        {
            if (type == 0)
            {
                VIT = pow(cos(diff_tetaa), 15) * lineaire.calcul(erreura);
            }
            else
                VIT = pow(cos(diff_tetaa), 15) * lineaire.calcul(erreura);
        }

        else
        {
            VIT = cos(diff_tetaa) * lineaire.calcul(erreura);
        }
    }
    return constrain(VIT, -1.0,1.0);
}
float Polaire::arriere(float consigne, float sortie, float teta_act, float teta, int type)
{
    if (type == 0)
    {

        lineaire.setKd(0.05f);
        lineaire.setKp(0.015f);
    }
    else
    {

        lineaire.setKd(KDlin);
        lineaire.setKp(KPlin);
    }

    float diff_tetaaa = teta - teta_act;
    float VIT;
    diff_tetaaa = limit_angle(diff_tetaaa);
    float erreuraa = consigne - sortie;
    if (consigne < 20.0 || consigne > 30.0)
        lineaire.reset();
    if (type == 2)
    {
        VIT = pow(cos(diff_tetaaa), 55) * lineaire.calcul(erreuraa);
    }
    else
    {

        if (consigne > 30.0f)
        {
            if (type == 0)
            {
                VIT = pow(cos(diff_tetaaa), 15) * lineaire.calcul(erreuraa);
            }
            else
                VIT = pow(cos(diff_tetaaa), 15) * lineaire.calcul(erreuraa);
        }

        else
        {
            VIT = cos(diff_tetaaa) * lineaire.calcul(erreuraa);
        }
    }
    return constrain(VIT, -1.0, 1.0);
}
float Polaire::angle(float consigne, float sortie, float dist, int type)
{
    float diff_teta = consigne - sortie;
    float VIT;
    diff_teta = limit_angle(diff_teta);
    if ((fabs(limit_angle(consigne - sortie)) < (PI / 180.0)) || (fabs(limit_angle(consigne - sortie)) > (PI / 5.0))||type!=2)
        angulaire.reset();
    if (dist != 0)
    {
        VIT = (atan(dist / 50.0) / (PI / 2.0)) * angulaire.calcul(diff_teta);
        if (dist < 5.0)
            VIT = 00;
    }
    else
    {
        VIT = angulaire.calcul(diff_teta);
    }

    return constrain(VIT, -1.0, 1.0);
}