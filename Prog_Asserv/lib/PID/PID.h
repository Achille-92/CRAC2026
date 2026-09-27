#ifndef PID_H
#define PID_H

class PID
{
private:
    float Kp;
    float Kd;
    float Ki;
    float Max_integral;
    float erreur;
    float erreur_pre;
    float integral;
    float Max_sortie;

public:
    PID(float kp, float ki, float kd, float max_integral, float max_sortie);
    void init(float kp,float ki,float kd){Kp=kp,Ki=ki,Kd=kd;}
    float calcul(float erreur);
    void reset(void);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setMaxIntegral(float max_integral);
    void setMaxSortie(float max_sortie);
};

#endif