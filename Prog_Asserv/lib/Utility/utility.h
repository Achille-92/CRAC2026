#define KPlin 0.007f
#define KDlin 0.15f
#define KIlin 0.1f
#define KPang 1.7f          
#define KDang 30.0f   
#define KIang 100.0f      
#define KPdroit 700.0f
#define KIdroit 1.7f
#define KPgauche 700.0f
#define KIgauche 1.7f
#define KdVitesse 100.0f

int signe(float a);
int ensemble(float valeur, float delta);
int comparer(float valeur_comparer, float valeur_compareur, float delta);
float limit_angle(float variable);
float logistic(float x,float x0,float k);
float logistic_inverse(float x,float x0,float k);