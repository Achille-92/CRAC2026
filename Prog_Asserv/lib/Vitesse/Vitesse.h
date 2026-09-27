#include <PID.h>

class vitesse
{
private:
    PID droit_pid;
    PID gauche_pid;

public:
    vitesse();
    float droit(float consigne, float sortie,int type,float dist);
    float gauche(float consigne, float sortie,int type,float dist);
    void reset(void);
};