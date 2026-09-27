#include <PID.h>

class Polaire
{
private:
    PID lineaire;
    PID angulaire;

public:
    Polaire();
    float avant(float consigne, float sortie,float teta_act,float teta,int type);
    float arriere(float consigne, float sortie,float teta_act,float teta,int type);
    float angle(float consigne, float sortie,float dist,int type);
};